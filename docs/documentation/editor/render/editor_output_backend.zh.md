# Editor Output Backend

`src/rtr/editor/render/editor_output_backend.hpp` 是 editor runtime 使用的 swapchain 驱动 renderer backend。

它的职责包括：

- 持有 editor 侧可见的 `PipelineRuntime`
- 拥有并绑定 `EditorImGuiPass`
- 消费 `pipeline.final_output(...)`
- 把场景图像切到 UI 可采样 layout
- 把 editor UI 渲染到输出目标
- 通过 `IEditorInputCaptureSource` 暴露 ImGui capture 状态

这个 backend 是当前 editor 路径不再需要单独 editor-specific scene pipeline 的关键原因：场景仍由 `ForwardPipeline` 或 `ShaderToyPipeline` 负责，editor 合成发生在这里。
