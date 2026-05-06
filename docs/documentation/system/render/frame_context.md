# Frame Context

`src/rtr/system/render/frame_context.hpp` defines the per-frame recording context passed into every render pass and pipeline.

It exists to bundle the minimum state needed while recording a frame:

- the active `rhi::CommandBuffer`
- the active `rhi::Device`
- the render extent
- the current frame slot index
- an optional `RenderOutputTarget`

This is the bridge between pure content rendering and output backends. A pipeline can always render into offscreen images, while backend-owned passes can additionally access `output_target()` when a swapchain-style destination exists.

Current coupling:

- compatibility accessors `swapchain_image()` and `swapchain_image_view()` still exist for older pass code
- callers must check `has_output_target()` before treating the frame as presentable
