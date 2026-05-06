# 在 macOS 上分析 RTR2 性能

这篇文档说明如何从 `quickstart` 抓第一帧，并逐步定位慢 pass。

`RTR2` 是 Vulkan 渲染器。在 macOS 上它通过 MoltenVK 落到 Metal，所以最实用的流程是：

1. 用 MoltenVK 导出 `.gputrace`
2. 用 Xcode 打开这帧并看 GPU capture
3. 用 Instruments 的 `Metal System Trace` 验证时间到底花在哪里

## 1. 前置条件

- macOS，且已安装 Xcode
- 机器上可用 Vulkan SDK / loader
- 已经构建出 `quickstart`

如果还没构建：

```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug --target quickstart
```

先确认 Vulkan 最终确实走到了 MoltenVK：

```bash
vulkaninfo --summary
```

你应该能看到：

- `driverName = MoltenVK`
- `VK_EXT_metal_surface` 之类的 macOS / Metal surface 扩展

## 2. 先知道 `quickstart` 这一帧长什么样

可执行文件是：

```bash
./build/Debug/examples/quickstart
```

入口在 `examples/editor/quickstart.cpp`。

这个 sample 现在走的是“纯 `ForwardPipeline` + `EditorOutputBackend`”路径，而不是单独的 `ForwardEditorPipeline`。因此一帧大致分成：

1. `ForwardPass`：把 3D 场景渲染到离屏颜色目标
2. 图像 barrier：让离屏颜色目标转成可采样
3. `EditorOutputBackend`：把场景图像切到可采样状态，并准备输出目标
4. `EditorImGuiPass`：把编辑器 UI 画到 swapchain，上面的 scene view 会采样离屏图像

所以你在 Xcode 里看到的通常不是“一个场景 pass”，而是“场景 pass + UI pass”。

## 3. 抓第一帧

在仓库根目录执行：

```bash
cd /Users/jinceyang/Desktop/codebase/graphics/rtr2
```

运行：

```bash
MTL_CAPTURE_ENABLED=1 \
MVK_CONFIG_AUTO_GPU_CAPTURE_SCOPE=2 \
MVK_CONFIG_AUTO_GPU_CAPTURE_OUTPUT_FILE=/tmp/rtr2-quickstart-first-frame.gputrace \
./build/Debug/examples/quickstart
```

这几个环境变量的作用：

- `MTL_CAPTURE_ENABLED=1`：开启 Metal 抓帧
- `MVK_CONFIG_AUTO_GPU_CAPTURE_SCOPE=2`：让 MoltenVK 自动抓单帧
- `MVK_CONFIG_AUTO_GPU_CAPTURE_OUTPUT_FILE=...`：把结果写到文件

抓完后打开：

```bash
open /tmp/rtr2-quickstart-first-frame.gputrace
```

Xcode 会直接打开这份 trace。

## 4. 在 Xcode 里读这第一帧

在 Xcode 的 GPU Frame Capture 里按下面顺序看：

1. 先看整帧或 command buffer 概览
2. 找到离屏场景渲染的那个 render pass
3. 再找到后面画 ImGui / editor 的 render pass
4. 展开 encoder 和 draw call，看 GPU 时间集中在哪一段

对 `quickstart` 来说，可以这样和源码对应：

- 场景渲染耗时：
  `ForwardPipeline::render()` 里的 `m_forward_pass.execute(...)`
- barrier 段：
  `EditorOutputBackend::record_output(...)` 里把场景图像切到可采样状态的 barrier
- UI / overlay 耗时：
  `EditorOutputBackend::record_output(...)` 里调用的 `EditorImGuiPass::execute(...)`

如果某一个 encoder 明显比其它段长，那就是第一嫌疑 pass。

## 5. 怎么把慢段映射回源码

当 Xcode 里已经看出慢段之后，用下面这套判断。

### 场景 pass 慢

典型现象：

- 大部分 GPU 时间都花在 UI pass 之前
- draw 很多
- fragment 或带宽计数偏高

先看这些文件：

- `src/rtr/system/render/pipeline/forward/forward_pipeline.hpp`
- `src/rtr/system/render/pipeline/forward/forward_pass.hpp`
- `src/rtr/framework/integration/render/forward_scene_view_builder.hpp`

优先问这几个问题：

- 是不是 Retina 下离屏目标比预期大很多
- 提交的 mesh / draw item 是不是太多
- 对这个简单场景来说 fragment shading 是否过重
- 颜色/深度 attachment 是否在频繁重建

### 编辑器 / ImGui pass 慢

典型现象：

- 第二个 render pass 意外地很重
- 停靠、缩放、打开很多 panel 时成本明显上升

先看：

- `src/rtr/editor/render/editor_output_backend.hpp`
- `src/rtr/editor/render/editor_imgui_pass.hpp`

优先问这几个问题：

- scene panel 的贴图是不是被比预期更频繁地刷新
- UI overdraw 是否过高
- 缩小窗口或减少 editor 交互后，这段成本是否明显下降

### present 路径或帧尾同步慢

典型现象：

- GPU 真正工作时间不长，但帧时间还是很长
- present 附近有明显 idle gap

先看：

- `src/rtr/system/render/renderer.hpp`
- `src/rtr/system/render/frame_scheduler.hpp`

优先问这几个问题：

- 是不是卡在 acquire/present，而不是卡在着色
- 是否正好触发了 swapchain recreate
- validation 或抓帧工具本身是不是把时间拉长了

## 6. 用 Instruments 验证

Xcode 抓一帧适合“找嫌疑人”，Instruments 更适合确认“它是不是真的导致帧时间变长”。

步骤：

1. 打开 Instruments
2. 选择 `Metal System Trace`
3. 启动 `./build/Debug/examples/quickstart`
4. 录 3 到 5 秒，同时转动相机或者操作 editor
5. 停止录制，查看 CPU 和 GPU 时间线

重点看：

- 有没有和 Xcode 抓帧里同一个 pass 对应的长 GPU encoder
- CPU 有没有没及时喂 GPU
- GPU 有没有在等 present 或同步
- 改变 framebuffer 大小时有没有尖峰

如果 Instruments 里慢帧和单帧 capture 指向同一个 pass，这个热点就比较可信。

## 7. 减少误判

在真正下结论前，先控制这些变量：

- 性能结论尽量基于 `Release`
- `Debug` 的时间只能做方向判断
- 除非你就是在测 resize，否则录制时不要拖动窗口尺寸
- 每次测试尽量保持相同场景和相同交互

几个重要提醒：

- Debug 构建通常会开启 Vulkan validation，时间会失真
- Metal frame capture 本身就会带额外开销
- `quickstart` 自带 editor UI，它不是纯场景 benchmark

## 8. 一套实用的最小工作流

按这个循环做就够了：

1. 从 `quickstart` 抓一帧
2. 判断慢段属于 scene、barrier/sync 还是 ImGui
3. 打开对应源码
4. 提一个明确假设
5. 用 Instruments 复录，确认修改前后帧时间有没有变化

比较好的第一批假设通常是：

- Retina 导致离屏 scene target 太大
- scene pass 的 fragment 成本太高
- editor UI 掩盖了真正的 scene 成本
- 交互过程中触发了 swapchain recreate

## 9. 可选：抓 Vulkan API 级别 trace

如果你要的是 Vulkan API 级别回放，而不是 Apple GPU 工具链，可以用 `gfxreconstruct`：

```bash
VK_INSTANCE_LAYERS=VK_LAYER_LUNARG_gfxreconstruct \
GFXRECON_CAPTURE_FILE=/tmp/rtr2-quickstart.gfxr \
./build/Debug/examples/quickstart
```

它更适合做 Vulkan 调用流、资源生命周期和回放分析；但在 macOS 上做 pass 级性能定位，第一选择通常还是 Xcode 和 Instruments。
