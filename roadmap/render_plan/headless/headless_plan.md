# RTR2 Headless 离线渲染计划

## 动机

IPC 等物理模拟天然是离线密集型：343 顶点的 tet block 原版 IPC 每步 ~400ms，rtr2 实现在放宽容差后也只能达到 ~10 FPS。与其追求实时帧率，不如提供一个 headless 模式：

1. 无窗口、无 swapchain，纯 GPU 离线渲染到图片序列
2. 物理按固定 dt 逐帧推进，不受渲染帧率约束
3. 最终用 ffmpeg 合成视频

## 当前架构分析

### 已有优势

- `ForwardPipeline` 内部已经是 **offscreen 渲染**：先渲染到独立的 color/depth image，再 blit 到 swapchain
- offscreen image 已标记 `eTransferSrc`，可以直接 readback
- 物理系统 (`PhysicsSystem`) 可以独立于渲染运行
- `AppRuntime` 是 callback 驱动的，控制流可定制

### 需要解耦的耦合点

| 耦合点 | 位置 | 问题 |
|--------|------|------|
| Window 创建 | `rhi/window.hpp` | GLFW 必须创建可见窗口 |
| Surface 要求 | `rhi/context.hpp:create_surface()` | Vulkan Context 构造必须传入 surface creator |
| Swapchain 获取/呈现 | `render/frame_scheduler.hpp` | `begin_frame()` acquire + `submit_and_present()` present 是强制流程 |
| 主循环退出条件 | `app_runtime.hpp:171` | `!window().is_should_close()` |
| 事件轮询 | `app_runtime.hpp:173` | `window().poll_events()` |

## 架构设计

### 方案：分层抽取，最小侵入

不修改现有实时路径，而是在其旁边建一条平行的 headless 路径。

```text
                    ┌──────────────────────────────────────────┐
                    │           HeadlessRuntime                │
                    │  (替代 AppRuntime 的离线主循环)            │
                    │                                          │
                    │  for frame in 0..N:                      │
                    │    1. physics.step(dt)                   │
                    │    2. pipeline.prepare_frame(scene)      │
                    │    3. headless_scheduler.begin_frame()   │
                    │    4. pipeline.render(frame_ctx)         │
                    │    5. readback offscreen → staging buf   │
                    │    6. save PNG to output_dir/            │
                    │    7. headless_scheduler.end_frame()     │
                    └──────────────────────────────────────────┘

RHI 层新增:
  ┌───────────────┐   ┌─────────────────────┐
  │ HeadlessWindow│   │ HeadlessScheduler   │
  │ (无 GLFW)     │   │ (无 swapchain)       │
  │ stub 接口     │   │ 管理 cmd buf + fence │
  └───────────────┘   └─────────────────────┘
```

## 实施阶段

---

### Phase 0: Window 接口抽象

**目标**：让 Window 可以是 headless stub，不创建 GLFW 窗口。

**修改文件**：
- `src/rtr/rhi/window.hpp`

**做法**：
1. 抽取 `IWindow` 接口（或给现有 `Window` 加一个 headless 构造路径）：
   - `framebuffer_size()` → 返回配置的固定分辨率
   - `is_should_close()` → 始终返回 false（由外部控制帧数）
   - `poll_events()` → no-op
   - `create_vk_surface()` → 返回 `VK_NULL_HANDLE` 或不调用
2. 保持现有 GLFW 路径不变

**预计改动量**：~50 行新增，0 行现有代码修改

---

### Phase 1: Context 支持无 Surface 创建

**目标**：允许 Vulkan Context 在没有 surface 的情况下初始化。

**修改文件**：
- `src/rtr/rhi/context.hpp`

**做法**：
1. `ContextCreateInfo` 增加 `bool headless = false`
2. `create_instance()` 中 headless 模式跳过 presentation 相关 extension
3. `create_surface()` 在 headless 模式下跳过
4. Device 选择时不要求 present queue（headless 只需 graphics + transfer queue）

**预计改动量**：~30 行修改

---

### Phase 2: HeadlessFrameScheduler

**目标**：替代 `FrameScheduler`，不走 swapchain acquire/present，直接管理 command buffer 提交和 fence 同步。

**新增文件**：
- `src/rtr/rhi/headless_frame_scheduler.hpp`

**接口**：
```cpp
struct HeadlessFrameScheduler {
    // 初始化 N 个 in-flight frame 的 command buffer + fence
    explicit HeadlessFrameScheduler(const Device& device, uint32_t frames_in_flight = 2);

    // 等待上一帧 fence，重置 command buffer，返回可录制的 cmd
    FrameContext begin_frame();

    // 提交 command buffer，signal fence
    void end_frame(FrameContext& ctx);

    // 当前帧的 command buffer（供 pipeline 使用）
    vk::CommandBuffer command_buffer() const;
};
```

**要点**：
- 不需要 swapchain image index
- Fence 同步确保 GPU 完成后再 readback
- 复用现有 `FrameContext` 结构，pipeline 代码零改动

**预计改动量**：~120 行新增

---

### Phase 3: Image Readback 工具

**目标**：将 offscreen color image 从 GPU 读回 CPU 并保存为 PNG。

**新增文件**：
- `src/rtr/rhi/image_readback.hpp`

**接口**：
```cpp
// 将 GPU image 拷贝到 staging buffer，返回 CPU 可读的像素数据
std::vector<uint8_t> readback_image(
    const Device& device,
    vk::CommandBuffer cmd,
    vk::Image source,
    vk::Format format,
    uint32_t width, uint32_t height);

// 保存 RGBA8 像素数据为 PNG
void save_png(const std::string& path,
              const uint8_t* pixels,
              uint32_t width, uint32_t height);
```

**实现要点**：
1. 创建 host-visible staging buffer
2. `vkCmdCopyImageToBuffer` 从 offscreen image → staging
3. fence 等待后 map buffer 读取像素
4. 用 `stb_image_write.h` 写 PNG（项目已有 stb 依赖）

**预计改动量**：~100 行新增

---

### Phase 4: HeadlessRuntime 主循环

**目标**：组装 headless 模式的完整主循环。

**新增文件**：
- `src/rtr/app/headless_runtime.hpp`

**接口**：
```cpp
struct HeadlessConfig {
    uint32_t width = 1920;
    uint32_t height = 1080;
    uint32_t total_frames = 500;
    double fixed_delta_seconds = 0.01;
    std::string output_dir = "output/frames";
    std::string filename_pattern = "frame_{:06d}.png";  // fmt 格式
};

struct HeadlessRuntime {
    explicit HeadlessRuntime(HeadlessConfig config);

    // 用户设置 scene、camera、pipeline（与 AppRuntime 相同的 API）
    void set_pipeline(std::unique_ptr<RenderPipeline> pipeline);
    World& world();

    // 运行全部帧，逐帧输出 PNG
    HeadlessResult run();
};
```

**主循环伪代码**：
```cpp
HeadlessResult HeadlessRuntime::run() {
    for (uint32_t frame = 0; frame < m_config.total_frames; ++frame) {
        // 1. 物理推进
        step_scene_physics(*scene, m_physics, m_config.fixed_delta_seconds);

        // 2. 准备渲染数据
        m_pipeline->prepare_frame(prepare_ctx);

        // 3. 录制 GPU 命令
        auto frame_ctx = m_scheduler.begin_frame();
        m_pipeline->render(frame_ctx);

        // 4. 录制 readback 命令（在同一 cmd buf 里）
        record_readback(frame_ctx.command_buffer, offscreen_image);
        m_scheduler.end_frame(frame_ctx);

        // 5. GPU 完成后，保存图片
        save_png(format("frame_{:06d}.png", frame), staging_data, w, h);

        // 6. 进度输出
        if (frame % 10 == 0)
            log::info("Frame {}/{}", frame, total_frames);
    }
    return {.ok = true, .frames_written = total_frames};
}
```

**预计改动量**：~150 行新增

---

### Phase 5: Headless Example 与 ffmpeg 集成

**目标**：提供一个完整的离线渲染 example 和视频合成脚本。

**新增文件**：
- `examples/headless/ipc_headless_render.cpp`
- `examples/headless/make_video.sh`

**Example 结构**：
```cpp
int main() {
    HeadlessConfig config{
        .width = 1920, .height = 1080,
        .total_frames = 500,          // 5 秒 @ dt=0.01
        .fixed_delta_seconds = 0.01,
        .output_dir = "output/ipc_block_headless",
    };

    HeadlessRuntime runtime(config);

    // 设置 scene（与 ipc_fixed_end_block_editor 相同的场景配置）
    auto& scene = runtime.world().create_scene("headless_scene");
    setup_camera(scene, config.width, config.height);
    setup_lights(scene);
    setup_ground(scene, runtime);
    setup_ipc_block(scene, runtime);

    // 使用 ForwardPipeline（不是 EditorPipeline，不需要 ImGui）
    runtime.set_pipeline(std::make_unique<ForwardPipeline>(...));

    runtime.run();
}
```

**视频合成脚本**：
```bash
#!/bin/bash
# make_video.sh <frame_dir> [output.mp4] [fps]
ffmpeg -framerate ${3:-30} -i "$1/frame_%06d.png" \
       -c:v libx264 -pix_fmt yuv420p -crf 18 \
       "${2:-output.mp4}"
```

---

## 改动总结

| Phase | 新增文件 | 修改文件 | 新增行数 | 修改行数 |
|-------|----------|----------|----------|----------|
| 0 - Window 抽象 | 0 | `rhi/window.hpp` | ~50 | ~10 |
| 1 - Context headless | 0 | `rhi/context.hpp` | ~10 | ~20 |
| 2 - HeadlessScheduler | `rhi/headless_frame_scheduler.hpp` | 0 | ~120 | 0 |
| 3 - Image Readback | `rhi/image_readback.hpp` | 0 | ~100 | 0 |
| 4 - HeadlessRuntime | `app/headless_runtime.hpp` | 0 | ~150 | 0 |
| 5 - Example + Script | `examples/headless/*` | `examples/CMakeLists.txt` | ~100 | ~5 |
| **合计** | **4 个新文件** | **3 个文件** | **~530 行** | **~35 行** |

## 依赖

- `stb_image_write.h`（项目已有 stb）
- `ffmpeg`（仅视频合成，不编译进项目）

## 不做的事

- 不修改现有实时渲染路径的任何行为
- 不引入新的第三方库
- 不做 headless 模式下的 UI / 交互
- 不做多 GPU 分布式渲染

## 建议实施顺序

Phase 0-1 可以一起做（~1 小时），Phase 2-3 一起做（~2 小时），Phase 4 依赖前面完成（~1 小时），Phase 5 最后收尾（~30 分钟）。总计 **约半天工作量**。
