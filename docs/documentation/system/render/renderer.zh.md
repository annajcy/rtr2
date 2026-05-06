# Renderer

`src/rtr/system/render/renderer.hpp` 定义了当前用于 realtime、editor 和 preview-style offline 路径的 swapchain 驱动 renderer 引导层。

这里有三层关键结构：

- `IRenderer`：面向 runtime 的小接口
- `RendererT<TBackend>`：按 output backend 参数化的模板 renderer
- `ComputeJob`：一次性异步 compute 工作的辅助对象

`RendererT` 当前直接持有：

- `rhi::Window`
- `rhi::Context`
- `rhi::Device`
- `FrameScheduler`
- 一个活动中的 `RenderPipeline`
- 一个 backend 实例

它的主要职责是：

- 构建 `PipelineRuntime`
- 在绑定后保持 pipeline 不可替换
- 把 swapchain 状态变化同步给 pipeline
- 把最终输出处理委托给所选 backend

当前耦合点：

- bootstrap 仍然是 window + surface + swapchain 驱动
- `draw_frame()` 目前只适用于满足当前 swapchain 契约的 backend
