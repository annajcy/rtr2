# Frame Scheduler

`src/rtr/system/render/frame_scheduler.hpp` 是 realtime 和 preview 渲染路径里的 swapchain 驱动帧调度层。

它的职责是：

- 持有 swapchain
- 持有每帧 command buffer、semaphore 和 fence
- acquire 下一张 swapchain image
- 提交已录制的命令并 present
- 在 resize 或 out-of-date 事件后重建 swapchain 资源

这里几个核心数据类型是：

- `PerFrameResources`：command buffer + acquire semaphore + in-flight fence
- `PerImageResources`：render-finished semaphore
- `FrameTicket`：当前帧的 frame slot、image slot 和 command buffer
- `SwapchainState`：generation、extent、image 数量和格式

这个类本身就是带 present 语义的，不是通用 frame executor。当前 headless 规划也把它视为 realtime / preview 专用工具，而不是未来 v1 headless 的基础抽象。
