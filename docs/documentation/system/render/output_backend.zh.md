# Output Backend

`src/rtr/system/render/output_backend.hpp` 定义了场景渲染完成后，pipeline 的最终 offscreen image 如何被消费。

当前契约是：

- backend 实现 `begin_frame()`
- pipeline 在 `FrameContext` 上完成内容渲染
- backend 在 `record_output(...)` 里消费 `pipeline.final_output(...)`
- backend 在 `end_frame(...)` 里完成提交和 present/export

关键类型包括：

- `RenderFrameTicket`：backend 持有的逐帧录制票据
- `RenderOutputBackendConcept`：`RendererT` 使用的编译期约束
- `RenderBackendServices`：backend 共享的 device/context/window/frame scheduler 服务
- `SwapchainFrameOutputBackendBase`：swapchain 路径的辅助基类

这个文件里的具体 backend 有：

- `SwapchainOutputBackend`：把最终 offscreen image 输出到 swapchain
- `OfflineImageOutputBackend`：仍沿用 swapchain preview 路径，但可选同步导出 PNG

当前耦合点：

- `RenderBackendServices` 仍然带有 `rhi::Window&` 和 `FrameScheduler&`
- `OfflineImageOutputBackend` 是 preview-capable 的 offline backend，不是真正无 surface 的 headless backend
