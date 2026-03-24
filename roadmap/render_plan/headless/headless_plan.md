# RTR2 Headless / Offline Render Plan

## 目标

为 RTR2 增加一条适合 IPC 等离线物理模拟的渲染路径，满足下面三个需求：

1. 物理模拟按固定 `dt` 推进，不受实时渲染帧率影响
2. 每个输出帧稳定导出为图片序列，最终由 `ffmpeg` 合成为视频
3. 尽量复用现有 `AppRuntime`、`Renderer`、`ForwardPipeline`，避免一次性重写整条渲染初始化链

---

## 设计原则

### 1. 先做“可用的离线导出”，再做“真正的无窗口 backend”

现有代码已经具备一个关键前提：`ForwardPipeline` 先渲染到 offscreen image，再输出到 swapchain。  
这意味着离线导出不需要先把整套 Vulkan 初始化改成真正的 headless，完全可以先把 offscreen 结果读回并导出。

因此实施分两层：

- `v0`：保留 GLFW / surface / swapchain，使用不可交互但非最小化窗口完成离线序列导出
- `v1`：在 `v0` 跑通后，再把 output backend 抽象到真正的无 surface / 无 swapchain headless 模式

这样可以显著降低一次性修改 `Context + Device + Renderer + Pipeline` 带来的风险。

### 2. 优先抽象“输出后端”，不是“Window”

现有系统真正的硬耦合，不在 `Window` 本身，而在下面三处：

- `Renderer::draw_frame()` 绑定了 acquire / render / present 整条流程
- `FrameContext` 强制携带 swapchain image/view
- `ForwardPipeline` 无条件执行 `PresentPass`

也就是说，真正要抽的是“本帧渲染结果输出到哪里”，而不是先把窗口类抽成 `IWindow`。

本计划里的职责边界进一步明确为：

- `Pipeline` 只负责产出 final offscreen image
- backend 负责把 final image 输出到具体目标
  - realtime: 输出到 swapchain present 路径
  - realtime editor: 输出到 editor 目标与 UI 合成路径
  - headless: readback/export 到磁盘

### 3. headless 必须复用实时路径的 update 语义

现有 `AppRuntime` 不只是 physics loop，它还负责：

- `world.fixed_tick(...)`
- `step_scene_physics(...)`
- `world.tick(...)`
- `world.late_tick(...)`
- `resources.tick(...)`

如果 headless 只保留 physics step，会和实时路径产生行为偏差，导致动画、控制器、资源生命周期、脚本逻辑失真。  
因此离线路径必须复用同一套 frame advance 语义，而不是另写一个缩水版主循环。

但这不意味着 `OfflineRuntime` 必须完整复刻 `AppRuntime` 的宿主能力。  
`OfflineRuntime` 默认不处理 input，并且 callback 只保留最小集接口，避免把 editor / realtime 宿主语义整块搬过来。

### 4. realtime 的 fixed-step 补偿机制，不应直接搬到 headless

当前 realtime runtime 的 fixed-step 逻辑，本质上是：

- 用墙钟时间计算 `frame_delta`
- 用 `accumulator += frame_delta` 追踪尚未模拟的时间
- 当 `accumulator >= fixed_delta_seconds` 时重复执行 fixed tick / physics step
- 用 `max_frame_delta_seconds` 和 `max_fixed_steps_per_frame` 限制单帧追赶量

这套机制的目的，是在实时渲染中吸收墙钟时间抖动，并避免 spiral of death。  
它服务的是“真实时间不可控”这个问题，不是 fixed-step 本身。

headless runtime 不受墙钟时间驱动，因此：

- 仍然需要 fixed-step 语义
- 不需要 accumulator 式的墙钟补偿
- 不需要 `max_frame_delta_seconds`
- 通常也不需要 `max_fixed_steps_per_frame` 作为运行时保护

headless 更合理的策略是显式指定：

- `sim_dt`
- `steps_per_output_frame`
- `total_output_frames`

然后每个输出帧按确定的次数推进 simulation。  
也就是说，headless 应该是“时间表驱动”，而不是“墙钟追赶驱动”。

---

## 当前架构中的真实约束

### 已有优势

- `ForwardPipeline` 已经先写入 offscreen color/depth，再通过 `PresentPass` 输出
- offscreen color image 已包含 `eTransferSrc` usage，可以作为 readback 源
- `InputSystem` 支持空事件源，不要求一定绑定真实窗口事件
- `AppRuntime` 的主循环逻辑已经比较清晰，适合抽取共享的 per-frame advance

### 当前硬耦合点

| 耦合点 | 位置 | 当前问题 |
|--------|------|----------|
| output 流程绑定到 `Renderer::draw_frame()` | `src/rtr/system/render/renderer.hpp` | acquire / render / present 是一个不可拆的方法 |
| `FrameContext` 强依赖 swapchain target | `src/rtr/system/render/frame_context.hpp` | 无法在没有 swapchain image/view 的情况下复用 |
| `ForwardPipeline` 固定执行 `PresentPass` | `src/rtr/system/render/pipeline/forward/forward_pipeline.hpp` | 无法直接把 offscreen 结果改为 readback/export |
| Vulkan 初始化要求 surface | `src/rtr/rhi/context.hpp`, `src/rtr/rhi/device.hpp` | 真正 headless 时，`Context` 和 `Device` 现在都默认有 present surface |
| runtime 退出与事件轮询绑定窗口 | `src/rtr/app/app_runtime.hpp` | 现有 `run()` 是 window-driven loop，不适合离线帧数驱动 |

### 一个重要结论

不要假设“只加一个 `HeadlessFrameScheduler` 就能 pipeline 零改动”。  
在现状下，哪怕 scheduler 不再 acquire/present，`ForwardPipeline` 仍然会访问 `ctx.swapchain_image()` 并执行 `PresentPass`。  
所以最小可行改动必须覆盖：

1. `Renderer` 的输出组织方式
2. `FrameContext` 的 render target 表达方式
3. `ForwardPipeline` 最后一步输出策略

另一个重要结论是：  
不要把 realtime 的 accumulator loop 整块复制到 headless。真正应该复用的是“单个 fixed step / 单个 frame update 的语义”，而不是“基于墙钟时间计算本帧要补多少步”的策略。

---

## 目标架构

### 总体思路

把现有流程拆成两层：

1. `FrameAdvance`  
   负责 world / physics / resource 的推进，决定“这一帧场景状态是什么”

2. `RenderOutputBackend`  
   负责 GPU 命令录制与最终输出，决定“这一帧渲染结果输出到哪里”

```text
AppRuntime / OfflineRuntime
        │
        ├── frame_time_policy -> frame_execution_plan
        │
        ├── frame_stepper.execute(plan)
        │      ├── world.fixed_tick
        │      ├── physics step
        │      └── world.tick / late_tick
        │
        ├── runtime tail
        │      └── resources.tick
        │
        └── render_one_frame()
               ├── pipeline.prepare_frame(...)
               ├── output_backend.begin_frame()
               ├── pipeline.render(frame_ctx)
               ├── output_backend.finalize_frame(...)
               └── output_backend.end_frame()
```

这里的 `FrameAdvance` 需要继续再拆成两部分：

1. `FrameTimePolicy`
   - 决定这一帧应该推进多少 simulation 时间
   - realtime: 来自墙钟时间 + accumulator + catch-up cap
   - headless: 来自配置好的 `sim_dt * steps_per_output_frame`

2. `FrameStepper`
   - 执行具体的 `fixed_tick / physics / tick / late_tick`
   - 不关心时间是怎么计算出来的

这层拆分是本次重构的关键。  
否则 headless 很容易被迫复用 realtime 的墙钟补偿逻辑，语义会不干净。

这里进一步明确一个实现决策：

- 第一版不引入 `traits` 或 runtime 级别的模板分发
- 第一版先做两个 `FrameTimePolicy`
  - `RealtimeFrameTimePolicy`
  - `OfflineFrameTimePolicy`
- 第一版先做一个共享的 `FrameStepper`
- `FrameTimePolicy` 与 `FrameStepper` 之间通过统一的 `FrameExecutionPlan` 交互

这样可以把变化点收敛在“时间如何计算”，而不是把整套 runtime 都模板化。

### 输出后端分三类

- `SwapchainOutputBackend`
  - 现有实时路径
  - acquire swapchain image
  - backend 负责把 final offscreen image 输出到 swapchain
  - submit + present

- `EditorOutputBackend`
  - realtime editor 路径
  - backend 负责把 final offscreen image 接到 editor scene target / UI 合成路径
  - 保持 editor 专有输出语义，不混入 headless

- `OfflineImageOutputBackend`
  - 离线路径
  - 不要求 present 到屏幕
  - backend 从 final offscreen image 做 readback
  - backend 负责 export
  - fence 完成后交给 CPU 写 PNG

### Pipeline 侧的变化

`ForwardPipeline` 不再把“最终一定 blit 到 swapchain”写死。  
更合理的拆法是：

- `ForwardPass`
  - 只负责把 3D 场景渲染到 offscreen color/depth

- `FinalOffscreenImage`
  - pipeline 负责把场景渲染结果收敛到可供 backend 消费的 final offscreen image
  - pipeline 不负责 readback/export
  - pipeline 不负责 present

这样 `ForwardPipeline` 的核心场景渲染部分保持不变，差异只留在最后一步。

---

## 实施方案

## Phase A: 抽取共享的 Frame Advance 语义

### 目标

把 `AppRuntime::run()` 中“更新世界状态”的部分抽出来，形成实时与离线共用的 per-frame 逻辑，同时把“时间如何推进”与“推进时实际做什么”分开。

### 需要解决的问题

当前 `AppRuntime` 的主循环把下面几件事写死在一个 while 里：

- 输入 begin/end
- 事件轮询
- fixed tick
- physics step
- frame tick / late tick
- pipeline prepare
- renderer draw
- resources tick

这导致 headless runtime 很容易复制一份相似但不完全一致的逻辑，后续维护风险很高。

更具体地说，现在有两类逻辑被混在一起：

1. 时间策略
   - `frame_delta = now - previous_time`
   - `accumulator += frame_delta`
   - `max_frame_delta_seconds`
   - `max_fixed_steps_per_frame`

2. 真正的 update 语义
   - `world.fixed_tick(...)`
   - `step_scene_physics(...)`
   - `world.tick(...)`
   - `world.late_tick(...)`
   - `resources.tick(...)`

重构时应当拆开这两类逻辑。

### 建议改法

建议至少抽成两层辅助接口。

第一层是“执行一个 fixed step”：

```cpp
void run_one_fixed_step(..., double fixed_dt, std::uint64_t fixed_tick_index);
```

第二层是“执行一个 frame update”：

```cpp
struct FrameExecutionPlan {
    uint32_t fixed_steps_to_run{0};
    double fixed_dt{0.0};
    double frame_delta_seconds{0.0};
};
```

然后再分别由 realtime/headless 构建自己的 `FrameExecutionPlan`。

这里建议显式避免过度设计：

- 不先引入 `Traits`
- 不先做 `template <typename PolicyTraits> Runtime`
- 不为 realtime/headless 各写一套 `FrameStepper`

当前真正变化的是时间策略，不是 step 语义本身。  
因此“一套统一 plan + 两个 policy + 一个 stepper”更适合现在的代码结构。

同时，当前版本的 `FrameExecutionPlan` 不需要额外的控制位，例如：

```cpp
bool skip_variable_update;
```

这类字段属于未来扩展点，不是当前 headless/realtime 分歧的核心。  
第一版建议保持 `FrameExecutionPlan` 最小化，只保留：

- `fixed_steps_to_run`
- `fixed_dt`
- `frame_delta_seconds`

如果更偏向函数式组织，也可以引入共享辅助函数，名字可以是下面任一风格：

- `advance_runtime_frame(...)`
- `step_runtime_frame(...)`
- `run_frame_update(...)`

它至少负责：

```cpp
struct FrameAdvanceParams {
    double frame_delta_seconds;
    double fixed_delta_seconds;
    uint32_t max_fixed_steps_per_frame;
    bool paused;
};

struct FrameAdvanceResult {
    uint32_t fixed_steps_executed{0};
};
```

但对于 headless，还建议补一个更直接的执行计划结构：

```cpp
struct OfflineFramePlan {
    double sim_dt{0.01};
    uint32_t steps_per_output_frame{1};
    double output_frame_delta_seconds{0.01};
};
```

共享执行逻辑需要覆盖：

1. `world.fixed_tick(...)`
2. `step_scene_physics(...)`
3. `world.tick(...)`
4. `world.late_tick(...)`

runtime loop 负责：

1. 是否处理 input begin/end
2. 是否进行事件轮询
3. `m_resources.tick(...)`

这里明确约定：

- `AppRuntime` 继续负责 input begin/end、event polling、`resources.tick(...)`
- `OfflineRuntime` 默认不处理 input，也不依赖 event polling
- `OfflineRuntime` 仍在 frame tail 负责 `resources.tick(...)`

### realtime 与 headless 的职责边界

realtime 负责：

- 从墙钟计算 `frame_delta`
- 更新 `accumulator`
- 决定本帧补多少个 fixed step
- 对补偿量做 cap

headless 负责：

- 直接给出本帧固定要跑多少个 fixed step
- `frame_delta_seconds = sim_dt * steps_per_output_frame`
- 不进行墙钟追赶

### 对现有配置的处理建议

`AppRuntimeConfig` 保留现有字段：

- `fixed_delta_seconds`
- `max_fixed_steps_per_frame`
- `max_frame_delta_seconds`

但这些字段是 realtime 专用语义。

`OfflineRenderConfig` 不要直接复用这三个字段，而应改成：

- `sim_dt`
- `steps_per_output_frame`
- `total_output_frames`
- `output_fps`

必要时可以在注释中明确：

- `sim_dt` 对应 single fixed step 的 simulation delta
- `output_fps` 默认应等于 `1 / (sim_dt * steps_per_output_frame)`
- 当 `output_fps` 与 `sim_dt * steps_per_output_frame` 不一致时，默认视为异常配置，至少给 warning
- `steps_per_output_frame` 决定一张导出图跨越多少 simulation 时间

### 范围

- 修改：`src/rtr/app/app_runtime.hpp`
- 新增：可选 `src/rtr/app/runtime_frame_advance.hpp`
- 新增：可选 `src/rtr/app/frame_time_policy.hpp`
- 新增：可选 `src/rtr/app/frame_stepper.hpp`

### 验收标准

- 实时路径行为不变
- headless runtime 后续直接复用同一套 update 逻辑
- 任何 scene/controller/animation 不会因为离线路径而漏掉 tick
- realtime 仍保留 accumulator/catch-up 逻辑
- headless 不再依赖墙钟时间或 accumulator
- `AppRuntime` 与 `OfflineRuntime` 对 input / resources 的 ownership 清晰，不发生重复调用

---

## Phase B: 把 Renderer 从“实时呈现器”收敛成“可选输出目标的渲染器”

### 目标

将 `Renderer::draw_frame()` 从“固定 swapchain acquire/present”重构为“调用一个 output backend”。

### 当前问题

现在 `Renderer::draw_frame()` 本质上做了三件事：

1. 从 `FrameScheduler` 获取 frame ticket
2. 录制 pipeline render 命令
3. 把 swapchain image transition 到 present 并提交

这三件事被硬编码在一个方法中，不利于插入离线输出。

### 建议改法

先不要上复杂的继承层级，先做一个足够薄的 backend 抽象即可。

例如：

```cpp
struct RenderFrameTicket {
    uint32_t frame_index;
    rhi::CommandBuffer* command_buffer;
    vk::Extent2D render_extent;
    std::optional<RenderTargetHandle> final_target;
};

class RenderOutputBackend {
public:
    virtual ~RenderOutputBackend() = default;
    virtual std::optional<RenderFrameTicket> begin_frame() = 0;
    virtual void end_frame(RenderFrameTicket& ticket) = 0;
    virtual RenderOutputState output_state() const = 0;
};
```

然后：

- 现有实时路径实现为 `SwapchainOutputBackend`
- realtime editor 路径实现为 `EditorOutputBackend`
- 离线路径实现为 `OfflineOutputBackend`

### 第一阶段不要做的事

- 不要一上来把整个 `Renderer` 拆成很多多态对象
- 不要同时重构 compute path
- 不要在这一阶段处理真正的无 surface Vulkan 初始化

### 范围

- 修改：`src/rtr/system/render/renderer.hpp`
- 修改：`src/rtr/system/render/frame_scheduler.hpp`
- 新增：可选 `src/rtr/system/render/output_backend.hpp`

### 验收标准

- 实时路径仍然通过 swapchain 正常显示
- `Renderer` 能支持 realtime / editor / headless 三类 backend

---

## Phase C: 让 FrameContext / Pipeline 不再硬编码 swapchain target

### 目标

让 pipeline 可以在有 swapchain 和无 swapchain 两种模式下工作。

### 当前问题

现有 `FrameContext` 构造时必须传入：

- swapchain image view
- swapchain image

这会导致 offline backend 只能伪造一个 swapchain target，接口语义不干净。

### 建议改法

把 `FrameContext` 的最终输出目标抽象成可选字段。

例如：

```cpp
struct FrameOutputTarget {
    const vk::raii::ImageView* image_view{};
    const vk::Image* image{};
    vk::ImageLayout expected_final_layout{vk::ImageLayout::eUndefined};
};
```

`FrameContext` 中改为：

- 始终有 `device`
- 始终有 `command_buffer`
- 始终有 `render_extent`
- 可选有 `output_target`

同时提供：

- `has_output_target()`
- `output_target()`

### 对 pipeline 的要求

`ForwardPipeline` 不再默认一定执行 `PresentPass`。  
应当改为：

- pipeline 统一产出 final offscreen image
- realtime / editor / headless 由各自 backend 决定如何消费该 image

### 推荐实现方式

不对 `ForwardPipeline` 增加“是否 editor / 是否 realtime”的模式配置。  
直接统一成：

- pipeline 只负责生成 final output
- output backend 负责决定 present / editor compose / offline export

### 范围

- 修改：`src/rtr/system/render/frame_context.hpp`
- 修改：`src/rtr/system/render/pipeline/forward/forward_pipeline.hpp`
- 修改：`src/rtr/system/render/pass/present_pass.hpp`

### 验收标准

- `ForwardPipeline` 在 realtime / editor / headless 三种输出模式下都能工作
- 不需要伪造假的 swapchain image 才能离线渲染
- pipeline 不再拥有 readback/export 逻辑
- backend 成为唯一的输出动作 owner

---

## Phase D: 实现 v0 离线导出路径

### 目标

在保留 GLFW / surface / swapchain 初始化的前提下，支持固定帧数的离线图片序列导出。

这是第一阶段真正应该落地的 MVP。

### 为什么先做 v0

因为它能验证最关键的三件事：

1. scene / physics / camera 在固定 `dt` 下的行为是否稳定
2. offscreen image 的 readback/export 链路是否正确
3. 输出序列和 ffmpeg 工作流是否满足最终使用需求

在这三件事没验证前，直接推进真正的无 surface backend 风险偏高。

### v0 方案

#### Runtime

新增 `OfflineRuntime`，但不复制 `AppRuntime` 的更新语义；它只负责：

- 以 `total_output_frames` 驱动循环
- 为每个输出帧调用共享的 frame advance
- 调用 renderer 的 offline backend 进行渲染和导出
- 暴露最小集 callback，而不是完整复刻 `AppRuntime` callbacks
- 默认不处理 input

这里要明确一个设计决策：

- `OfflineRuntime` 不从 `now - previous_time` 计算 `frame_delta`
- `OfflineRuntime` 不维护 realtime 风格的 `accumulator`
- `OfflineRuntime` 每个输出帧显式执行 `steps_per_output_frame` 个 fixed step
- `OfflineRuntime` 传给 `world.tick(...)` / `late_tick(...)` / `prepare_frame(...)` 的 `frame_delta_seconds` 为：
  `sim_dt * steps_per_output_frame`
- `OfflineRuntime` 默认不做 `input.begin_frame()` / `input.end_frame()` / window event polling

#### Callback 策略

`OfflineRuntime` 只保留最小集接口，建议为：

- `on_startup`
- `on_pre_update`
- `on_post_update`
- `on_pre_render`
- `on_post_render`
- `on_shutdown`

默认不提供：

- `on_input`
- 任何依赖实时窗口事件的 callback 语义

#### 配置建议

不要只保留 `fixed_delta_seconds + total_frames`。  
推荐配置拆成：

```cpp
struct OfflineRenderConfig {
    uint32_t width = 1920;
    uint32_t height = 1080;

    double sim_dt = 0.01;
    uint32_t steps_per_output_frame = 1;
    uint32_t total_output_frames = 500;
    double output_fps = 30.0;

    std::filesystem::path output_dir = "output/frames";
    std::string filename_pattern = "frame_{:06d}.png";
    bool create_non_interactive_window = true;
};
```

这样可以明确区分：

- 模拟步长 `sim_dt`
- 每输出帧推进多少模拟步 `steps_per_output_frame`
- 视频采样率 `output_fps`

后续制作慢动作/加速视频时更灵活。

同时增加一致性约束：

```cpp
const double expected_fps = 1.0 / (sim_dt * steps_per_output_frame);
```

默认要求：

- `output_fps` 与 `expected_fps` 一致

若不一致：

- 至少打印 warning
- 更严格时可以直接拒绝启动

### 推荐伪代码

```cpp
for (uint32_t output_frame = 0; output_frame < config.total_output_frames; ++output_frame) {
    for (uint32_t i = 0; i < config.steps_per_output_frame; ++i) {
        run_one_fixed_step(config.sim_dt);
    }

    const double frame_delta = config.sim_dt * static_cast<double>(config.steps_per_output_frame);
    run_variable_update(frame_delta);

    pipeline->prepare_frame(... frame_delta ...);
    renderer.render_frame<OfflineImageOutputBackend>();

    resources.tick(frame_serial);
    ++frame_serial;
}
```

这里没有任何 accumulator 或 catch-up 逻辑。  
因为 headless 的 simulation timeline 是显式确定的，不需要“补偿”。
默认也没有任何 input begin/end 或 event polling。

#### Readback

第一版用最保守实现即可：

1. pipeline 渲染到 final offscreen image
2. `OfflineImageOutputBackend` 在同一帧内 copy image -> staging buffer
3. backend 提交并等待 fence
4. backend map staging buffer
5. backend 写 PNG

这版吞吐一般，但实现简单、最利于验证 correctness。

### v0 窗口策略

v0 的首选策略固定为：

- 创建不可交互但非最小化窗口

原因是：

- v0 仍依赖 GLFW / surface / swapchain
- 最小化窗口在部分平台上可能导致 framebuffer extent 变为 0
- “不可交互但非最小化”更容易保持稳定的 swapchain/extents 语义

这意味着：

- v0 不追求真正意义上的无窗口
- v0 优先追求 surface/swapchain 行为稳定
- 真正的无窗口运行放到 v1 处理

同时明确：

- `create_non_interactive_window = true` 是 v0 默认策略
- 在文档里明确：`v0` 的目标是功能正确，不是最大吞吐

### 范围

- 新增：`src/rtr/app/offline_runtime.hpp`
- 新增：`src/rtr/rhi/image_readback.hpp`
- 新增：`examples/headless/ipc_offline_render.cpp`
- 修改：`examples/CMakeLists.txt`

另外建议在这一阶段同步调整：

- 修改：`src/rtr/app/app_runtime.hpp`
  - 把 realtime 的 accumulator / catch-up 保留在 realtime 专用时间策略里
- 新增：可选 `src/rtr/app/frame_time_policy.hpp`
  - 明确区分 `RealtimeFrameTimePolicy` 与 `OfflineFrameTimePolicy`
- 新增：可选 `src/rtr/app/frame_stepper.hpp`
  - 放共享的 fixed step / variable step 执行语义
- 新增：可选 `src/rtr/editor/render/editor_output_backend.hpp`
  - 明确 editor 输出路径和普通 realtime/headless 分开

### 验收标准

- 可以稳定输出一组 PNG
- 视频合成流程完整
- 与实时渲染相比，画面结果一致或仅有可解释差异
- `OfflineRuntime` 在默认配置下完全不依赖 input
- `output_fps` 与 simulation 时间基不一致时会给出明确 warning 或报错

---

## Phase E: 优化 v0 的吞吐

### 目标

在功能跑通后，减少“每帧 GPU 等待 + PNG 压缩串行”带来的总耗时。

### 优化方向

#### 1. staging ring

不要每帧临时创建 staging buffer。  
改成按 `kFramesInFlight` 预分配：

- `color readback buffer[N]`
- `fence[N]`

每次复用对应槽位。

#### 2. writer thread

GPU readback 完成后，把 CPU 像素数据丢给后台线程写 PNG，避免 render thread 阻塞在 PNG 压缩。

#### 3. 支持导出原始帧

如果 PNG 压缩仍然太慢，可以提供可选模式：

- `png`
- `ppm`
- `raw rgba`

让用户自行选择空间与时间的折中。

### 注意

这些都属于性能优化，不应阻塞 `v0` 落地。

---

## Phase F: 实现 v1 真正 headless backend

### 目标

在 `v0` 已经验证离线导出工作流有效后，再去掉 GLFW / surface / swapchain 依赖，做真正的 headless Vulkan 初始化。

### 这一阶段才需要处理的内容

#### 1. Context 无 surface 模式

- `ContextCreateInfo` 支持可选 surface creator
- headless 模式下跳过 surface 创建
- headless 模式下裁剪 presentation 相关 instance extensions

#### 2. Device profile 分离

现有 `Device` 默认假设需要 swapchain / present surface。  
v1 需要把 device requirement 拆成 profile，例如：

- `RealtimePresentation`
- `HeadlessOffscreen`

需要区分：

- 是否要求 surface support
- 是否要求 swapchain extension
- 是否要求 swapchain maintenance 相关 feature/extension

#### 3. Headless output backend

真正的 `OfflineOutputBackend` 在 v1 中不再依赖 swapchain 初始化。  
它只需要：

- command pool / command buffer
- in-flight fence
- final offscreen render extent

### 为什么放到最后

因为这是对 Vulkan 初始化链最敏感的一部分。  
如果前面还没有用 `v0` 验证过 offline 工作流，这里很容易把“渲染架构问题”和“平台/驱动初始化问题”混在一起调。

### 范围

- 修改：`src/rtr/rhi/context.hpp`
- 修改：`src/rtr/rhi/device.hpp`
- 新增：`src/rtr/system/render/headless_output_backend.hpp`
- 修改：`src/rtr/system/render/renderer.hpp`

### 验收标准

- 在无可见窗口、无 swapchain 的情况下可以稳定输出图片序列
- realtime 路径行为不变
- 同一套 `ForwardPipeline` 仍可运行于 realtime / editor / headless 三种 backend

---

## 文件级改动建议

| 模块 | 建议动作 | 原因 |
|------|----------|------|
| `app_runtime.hpp` | 抽共享 frame advance，并把 accumulator 留在 realtime 时间策略里 | 避免 headless runtime 复制墙钟补偿逻辑 |
| `frame_time_policy.hpp` | 新增，可选 | 显式区分 realtime 的 catch-up 与 headless 的 schedule-driven 推进 |
| `frame_stepper.hpp` | 新增，可选 | 抽出共享的 fixed tick / physics / tick / late_tick 执行语义，不为不同 runtime 各写一套 stepper |
| `renderer.hpp` | 抽输出 backend / ticket | 把 acquire-render-present 链解耦 |
| `editor_output_backend.hpp` | 新增，可选 | 把 realtime editor 输出路径独立成第三类 backend |
| `frame_context.hpp` | 改成可选 output target | 允许 pipeline 运行在无 swapchain 模式 |
| `forward_pipeline.hpp` | 只产出 final offscreen image | 把最终输出动作从 pipeline 中彻底移除 |
| `present_pass.hpp` | 收敛为 realtime 专用 pass | 避免 offline 路径误依赖 swapchain |
| `image_readback.hpp` | 新增 | 封装 GPU image -> CPU pixels |
| `offline_runtime.hpp` | 新增 | 固定帧数驱动的离线导出入口，不使用 accumulator/catch-up，默认不处理 input |
| `context.hpp` / `device.hpp` | 放到 v1 再改 | 避免一开始就动最敏感的 Vulkan 初始化链 |

---

## 不建议采用的方案

### 1. 先抽 `IWindow`

这件事不是完全没价值，但它不是当前最小可行路径上的关键点。  
先抽 `Window` 很容易让重构发散到输入系统、GLFW 生命周期、事件订阅，却不能解决 pipeline 仍然依赖 swapchain 的根问题。

### 2. 直接新造一整条 `HeadlessRuntime + HeadlessFrameScheduler + HeadlessPipeline`

这会带来两类重复：

- runtime 逻辑重复
- pipeline 逻辑重复

后面功能演进时，realtime/headless 两条路径会越来越难保持一致。

### 3. 在第一阶段就做真正的无 surface Vulkan 初始化

这会把平台初始化问题、device feature/profile 问题、离线导出问题一次性叠在一起。  
对开发节奏不利。

---

## 推荐实施顺序

### Milestone 1: 跑通 v0 离线导出

1. 抽共享 frame advance
2. 改 `Renderer`，支持 realtime / editor / offline 三类 output backend
3. 改 `FrameContext` 和 `ForwardPipeline`，让 pipeline 只产出 final offscreen image
4. 加 readback + PNG 导出
5. 做一个 IPC headless example

目标：一条命令输出图片序列并生成视频

### Milestone 2: 优化离线导出吞吐

1. staging ring
2. writer thread
3. 可选原始帧输出

目标：减少总渲染时间，但不改变行为语义

### Milestone 3: 做真正的 v1 headless backend

1. `Context` 无 surface
2. `Device` profile 化
3. `OfflineOutputBackend` 去 swapchain 依赖

目标：完全无窗口运行，但不影响 v0 已验证的离线工作流

---

## 接口草案

这一节给出第一版建议接口。  
目标不是一次性定死所有细节，而是先把模块边界固定下来，避免实现过程中反复漂移。

### 1. Frame Execution Plan

建议由 `FrameTimePolicy` 统一产出，交给 `FrameStepper` 执行。

```cpp
struct FrameExecutionPlan {
    uint32_t fixed_steps_to_run{0};
    double fixed_dt{0.0};
    double frame_delta_seconds{0.0};
};
```

语义约定：

- `fixed_steps_to_run`
  - 本帧要执行多少次 fixed step
- `fixed_dt`
  - 每次 fixed step 使用的 delta time
- `frame_delta_seconds`
  - 本帧 variable update / render prepare 所看到的 delta

对 headless：

```cpp
fixed_steps_to_run = steps_per_output_frame;
fixed_dt = sim_dt;
frame_delta_seconds = sim_dt * steps_per_output_frame;
```

对 realtime：

- `fixed_steps_to_run` 由 accumulator + cap 计算
- `fixed_dt` 来自 `fixed_delta_seconds`
- `frame_delta_seconds` 来自本帧 realtime `frame_delta`

### 2. Frame Time Policies

建议做成普通类或小型状态对象，不先做 traits / template runtime。

```cpp
class RealtimeFrameTimePolicy {
public:
    explicit RealtimeFrameTimePolicy(
        double fixed_delta_seconds,
        uint32_t max_fixed_steps_per_frame,
        double max_frame_delta_seconds
    );

    FrameExecutionPlan make_plan(double current_time_seconds, bool paused);

    void reset(double current_time_seconds);

private:
    double m_fixed_delta_seconds{0.0};
    uint32_t m_max_fixed_steps_per_frame{0};
    double m_max_frame_delta_seconds{0.0};

    double m_previous_time_seconds{0.0};
    double m_accumulator{0.0};
    bool m_initialized{false};
};
```

```cpp
class OfflineFrameTimePolicy {
public:
    explicit OfflineFrameTimePolicy(
        double sim_dt,
        uint32_t steps_per_output_frame
    );

    FrameExecutionPlan make_plan(bool paused) const;

private:
    double m_sim_dt{0.0};
    uint32_t m_steps_per_output_frame{1};
};
```

语义约定：

- `RealtimeFrameTimePolicy` 持有内部状态
  - `previous_time`
  - `accumulator`
- `OfflineFrameTimePolicy` 无需依赖任何 clock
- `paused=true` 时：
  - `fixed_steps_to_run = 0`
  - `frame_delta_seconds = 0.0`

### 3. Frame Stepper

`FrameStepper` 只执行共享 update 语义，不拥有时间策略，也不负责 input / event polling / output backend。

```cpp
struct FrameStepperContext {
    framework::core::World& world;
    system::physics::PhysicsSystem& physics_system;
    std::uint64_t& fixed_tick_index;
    std::uint64_t frame_serial;
};
```

```cpp
class FrameStepper {
public:
    void run_fixed_steps(
        const FrameExecutionPlan& plan,
        FrameStepperContext& ctx
    ) const;

    void run_variable_update(
        const FrameExecutionPlan& plan,
        FrameStepperContext& ctx
    ) const;

    void execute(
        const FrameExecutionPlan& plan,
        FrameStepperContext& ctx
    ) const;
};
```

推荐语义：

- `run_fixed_steps(...)`
  - loop `fixed_steps_to_run`
  - 每次执行：
    - `world.fixed_tick(...)`
    - `step_scene_physics(...)`
- `run_variable_update(...)`
  - `world.tick(...)`
  - `world.late_tick(...)`
- `execute(...)`
  - 先 fixed，再 variable

如果后面确实需要只跑 fixed 或只跑 variable，再额外加模式位；第一版不提前设计。

### 4. Offline Runtime Config

建议把 headless / offline 配置显式独立出来，不复用 `AppRuntimeConfig`。

```cpp
struct OfflineRuntimeConfig {
    uint32_t width{1920};
    uint32_t height{1080};

    double sim_dt{0.01};
    uint32_t steps_per_output_frame{1};
    uint32_t total_output_frames{500};
    double output_fps{100.0};

    std::filesystem::path output_dir{"output/frames"};
    std::string filename_pattern{"frame_{:06d}.png"};

    bool create_non_interactive_window{true};
    bool warn_on_fps_mismatch{true};
    bool fail_on_fps_mismatch{false};
};
```

建议提供校验接口：

```cpp
struct OfflineConfigValidationResult {
    bool ok{true};
    std::string error_message{};
    std::vector<std::string> warnings{};
};

OfflineConfigValidationResult validate_offline_runtime_config(
    const OfflineRuntimeConfig& config
);
```

推荐校验逻辑：

```cpp
expected_fps = 1.0 / (sim_dt * steps_per_output_frame)
```

- 若 `abs(output_fps - expected_fps) > epsilon`
  - `warn_on_fps_mismatch=true` 时记录 warning
  - `fail_on_fps_mismatch=true` 时返回 error

### 5. Offline Runtime Callbacks

保持最小集，不复刻 realtime input callback。

```cpp
struct OfflineRuntimeContext {
    framework::core::World& world;
    resource::ResourceManager& resources;
    system::render::Renderer& renderer;
    system::physics::PhysicsSystem& physics_system;
    std::uint64_t frame_serial{0};
    double frame_delta_seconds{0.0};
    std::function<void()> request_stop{};
};
```

```cpp
struct OfflineRuntimeCallbacks {
    std::function<void(OfflineRuntimeContext&)> on_startup{};
    std::function<void(OfflineRuntimeContext&)> on_pre_update{};
    std::function<void(OfflineRuntimeContext&)> on_post_update{};
    std::function<void(OfflineRuntimeContext&)> on_pre_render{};
    std::function<void(OfflineRuntimeContext&)> on_post_render{};
    std::function<void(OfflineRuntimeContext&)> on_shutdown{};
};
```

### 6. Offline Runtime

建议保持接口风格尽量接近 `AppRuntime`，但不暴露 input/window 语义。

```cpp
struct OfflineRuntimeResult {
    bool ok{true};
    std::string error_message{};
    std::uint64_t frames_rendered{0};
    std::uint64_t fixed_ticks{0};
    std::uint64_t images_written{0};
};
```

```cpp
class OfflineRuntime {
public:
    explicit OfflineRuntime(OfflineRuntimeConfig config = {});

    framework::core::World& world();
    resource::ResourceManager& resource_manager();
    system::render::Renderer& renderer();
    system::physics::PhysicsSystem& physics_system();

    void set_pipeline(std::unique_ptr<system::render::RenderPipeline> pipeline);
    void set_callbacks(OfflineRuntimeCallbacks callbacks);

    void request_stop();
    bool stop_requested() const;

    OfflineRuntimeResult run();

private:
    OfflineRuntimeContext make_runtime_context(double frame_delta_seconds);
};
```

`run()` 推荐流程：

```cpp
OfflineRuntimeResult OfflineRuntime::run() {
    validate_config_or_throw();
    callbacks.on_startup(...);

    for frame in [0, total_output_frames):
        if stop_requested:
            break;

        plan = offline_time_policy.make_plan(paused=false);
        callbacks.on_pre_update(...);
        frame_stepper.execute(plan, ...);
        callbacks.on_post_update(...);

        pipeline->prepare_frame(... plan.frame_delta_seconds ...);
        callbacks.on_pre_render(...);
        renderer.render_frame<OfflineImageOutputBackend>();
        callbacks.on_post_render(...);

        resources.tick(frame_serial);
        ++frame_serial;

    callbacks.on_shutdown(...);
    renderer.device().wait_idle();
    resources.flush_after_wait_idle();
}
```

### 7. Render Output Backend Base

backend 负责“如何消费 final offscreen image”，不负责 scene update。

```cpp
struct RenderOutputTarget {
    const vk::raii::ImageView* image_view{};
    const vk::Image* image{};
    vk::ImageLayout expected_layout{vk::ImageLayout::eUndefined};
    vk::Extent2D extent{};
};
```

```cpp
struct RenderFrameTicket {
    uint32_t frame_index{0};
    rhi::CommandBuffer* command_buffer{};
    vk::Extent2D render_extent{};
    std::optional<RenderOutputTarget> output_target{};
    std::optional<uint32_t> swapchain_image_index{};
};
```

```cpp
template <typename T>
concept RenderOutputBackendConcept = requires(
    T& backend,
    RenderPipeline& pipeline,
    FrameContext& frame_ctx,
    RenderFrameTicket& ticket
) {
    { backend.begin_frame() } -> std::same_as<std::optional<RenderFrameTicket>>;
    { backend.record_output(pipeline, frame_ctx) } -> std::same_as<void>;
    { backend.end_frame(ticket) } -> std::same_as<void>;
};
```

### 8. Concrete Backends

建议分成三个实现：

```cpp
class SwapchainOutputBackend final {
public:
    explicit SwapchainOutputBackend(...);

    std::optional<RenderFrameTicket> begin_frame();
    void record_output(RenderPipeline& pipeline, FrameContext& frame_ctx);
    void end_frame(RenderFrameTicket& ticket);
};
```

```cpp
class editor::render::EditorOutputBackend final : public IEditorInputCaptureSource {
public:
    explicit EditorOutputBackend(...);

    std::optional<RenderFrameTicket> begin_frame();
    void record_output(RenderPipeline& pipeline, FrameContext& frame_ctx);
    void end_frame(RenderFrameTicket& ticket);
};
```

```cpp
class OfflineImageOutputBackend final {
public:
    explicit OfflineImageOutputBackend(
        rhi::Device& device,
        const OfflineRuntimeConfig& config
    );

    std::optional<RenderFrameTicket> begin_frame();
    void record_output(RenderPipeline& pipeline, FrameContext& frame_ctx);
    void end_frame(RenderFrameTicket& ticket);
};
```

这里的关键语义是：

- `begin_frame()`
  - 返回 command buffer / frame index / render extent
  - realtime/editor 可能提供 `output_target`
  - headless 可以不提供 swapchain 风格的 `output_target`
- `record_output()`
  - realtime: 持有 realtime present pass，把 pipeline final output 输出到 swapchain
  - editor: 持有 `EditorImGuiPass`，把 pipeline final output 作为 scene texture 进行 compose
  - headless: 预留 readback/export 接口
- `end_frame()`
  - realtime/editor: submit + present
  - headless: 后续阶段再加 submit + wait + readback + export

### 9. Renderer 接口草案

`Renderer` 不维护运行时 `OutputMode`，而是直接持有三种 backend，并通过模板参数分发。

```cpp
class Renderer {
public:
    Renderer(int width, int height, std::string title);

    void set_pipeline(std::unique_ptr<RenderPipeline> pipeline);
    void bind_editor_host(std::shared_ptr<editor::EditorHost> host);

    template <RenderOutputBackendConcept TBackend>
    TBackend& backend();

    template <RenderOutputBackendConcept TBackend>
    void render_frame();

    PipelineRuntime build_pipeline_runtime();
};
```

实现上保留一个共享 helper：

```cpp
template <RenderOutputBackendConcept TBackend>
void render_frame_with_backend(TBackend& backend);
```

### 10. Frame Context 草案

`FrameContext` 不再强依赖 swapchain image。

```cpp
class FrameContext {
public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        const vk::Extent2D& render_extent,
        uint32_t frame_index,
        std::optional<RenderOutputTarget> output_target = std::nullopt
    );

    rhi::CommandBuffer& cmd();
    const rhi::Device& device() const;
    const vk::Extent2D& render_extent() const;
    uint32_t frame_index() const;

    bool has_output_target() const;
    const RenderOutputTarget& output_target() const;
};
```

### 11. Forward Pipeline 草案

pipeline 只负责生成最终输出，不再区分 editor 版 / realtime 版。

```cpp
struct PipelineFinalOutput {
    TrackedImage color;
    vk::Extent2D extent{};
};

class ForwardPipeline final : public RenderPipeline {
public:
    explicit ForwardPipeline(const PipelineRuntime& runtime, const ForwardPipelineConfig& config = {});

    void prepare_frame(const FramePrepareContext& ctx) override;
    void render(FrameContext& ctx) override;

    PipelineFinalOutput final_output(uint32_t frame_index) const override;
};
```

同一原则适用于：

- `ShaderToyPipeline`
- `EditableShaderToyPipeline`

对应的 editor-specific pipeline：

- `ForwardEditorPipeline`
- `ShaderToyEditorPipeline`
- `EditableShaderToyEditorPipeline`

都应被删除，改为“纯 pipeline + `editor::render::EditorOutputBackend`”。

### 12. Image Readback 草案

`image_readback.hpp` 只做 GPU image -> CPU pixels，不掺杂 runtime 语义。

```cpp
struct ReadbackImageDesc {
    vk::Image image{};
    vk::Format format{vk::Format::eUndefined};
    vk::Extent2D extent{};
    vk::ImageLayout current_layout{vk::ImageLayout::eUndefined};
};
```

```cpp
struct ReadbackResult {
    std::vector<std::uint8_t> pixels;
    uint32_t width{0};
    uint32_t height{0};
    vk::Format format{vk::Format::eUndefined};
};
```

```cpp
ReadbackResult readback_image(
    rhi::Device& device,
    const ReadbackImageDesc& desc
);

void save_png(
    const std::filesystem::path& output_path,
    const ReadbackResult& image
);
```

第一版允许内部同步等待；性能优化放到 Phase E 再做。

---

## 预估工作量

| Milestone | 工作内容 | 预估 |
|-----------|----------|------|
| 1 | 架构抽取 + v0 跑通 | 1 到 2 天 |
| 2 | 吞吐优化 | 0.5 到 1 天 |
| 3 | 真正 headless backend | 1 到 2 天 |

如果目标只是“尽快渲染 IPC 序列并合成视频”，做到 Milestone 1 就已经够用了。

---

## 验证清单

### 功能正确性

- 离线路径每帧都按固定 `sim_dt` 推进
- world fixed tick / frame tick / late tick 都被执行
- `ForwardPipeline` 在 realtime / editor / headless 三种输出模式下都能渲染
- 导出的 PNG 序列能稳定合成为视频

### 行为一致性

- 同一场景下，realtime 与 offline 首帧画面一致
- 固定随机种子时，offline 多次运行结果一致
- 相机、动画、物理更新顺序与 realtime 路径一致

### 性能与资源

- 没有每帧泄漏 staging buffer / fence / image
- 长序列导出时内存占用稳定
- `resources.tick(...)` 与 flush 语义保持正常

---

## 配套脚本

```bash
#!/bin/bash
# make_video.sh <frame_dir> [output.mp4] [fps]
ffmpeg -framerate "${3:-30}" -i "$1/frame_%06d.png" \
       -c:v libx264 -pix_fmt yuv420p -crf 18 \
       "${2:-output.mp4}"
```

---

## 结论

更贴近现有代码结构的方案，不是“围绕 Window 新建一套平行 headless 世界”，而是：

1. 复用现有 update 语义
2. 抽出 renderer 的输出后端
3. 让 pipeline 的最后一步不再写死 swapchain present
4. 先用 `v0` 验证离线导出工作流
5. 最后再把 Vulkan 初始化推进到真正的无 surface headless

这样改动路径更稳，也更符合 RTR2 现在的代码组织。
