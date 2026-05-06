# Editor Output Backend

`src/rtr/editor/render/editor_output_backend.hpp` is the swapchain-backed renderer backend used by editor runtimes.

Its responsibilities are:

- hold the editor-facing `PipelineRuntime`
- own and bind `EditorImGuiPass`
- consume `pipeline.final_output(...)`
- transition the scene image into sampled layout for the UI
- render editor UI into the output target
- report ImGui capture state through `IEditorInputCaptureSource`

This backend is the key reason the current editor path no longer needs a separate editor-specific scene pipeline. The scene stays in `ForwardPipeline` or `ShaderToyPipeline`; editor composition happens here.
