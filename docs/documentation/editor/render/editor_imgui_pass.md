# Editor ImGui Pass

`src/rtr/editor/render/editor_imgui_pass.hpp` records the actual ImGui/editor draw pass.

Its job is to:

- manage the `rhi::ImGuiContext`
- keep per-frame scene texture descriptors alive
- expose editor services such as scene texture ID, viewport rect, hover/focus state, and gizmo usage
- render ImGui draw data into the current output target

The pass is fed by `EditorOutputBackend`, which supplies:

- the sampled scene image view/layout
- the final output target
- scene viewport resize callbacks back into the content pipeline

Current coupling:

- the pass is tightly tied to ImGui Vulkan descriptor lifetime
- swapchain recreation currently forces scene texture descriptor refresh
