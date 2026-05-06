# Render Pipeline

`src/rtr/system/render/render_pipeline.hpp` 定义了所有纯内容 pipeline 共享的基类契约。

当前契约有三个关键点：

- `render(FrameContext&)`：录制 pipeline 的 GPU 工作
- `final_output(frame_index)`：暴露最终 offscreen image，供 backend 消费
- `prepare_frame(FramePrepareContext)`：在录命令前更新 pipeline 侧场景数据

配套类型包括：

- `PipelineRuntime`：构造期注入的 device、context、格式和 shader root 等服务
- `FramePrepareContext`：prepare 阶段可见的 world/resource/input 状态
- `SwapchainChangeSummary`：描述 swapchain 变化的 diff 对象
- `SceneViewportResizeEvent`：场景目标需要感知 viewport 尺寸时使用的事件

当前耦合点：

- `PipelineRuntime` 仍然携带 `rhi::Window&`
- 基类仍追踪来自 swapchain 的 extent 和 format，用于 rebuild 逻辑

尽管如此，所有权划分已经比较明确：pipeline 只产出最终图像；present 或 export 由 output backend 决定。
