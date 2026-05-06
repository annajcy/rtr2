# Render Resource State

`src/rtr/system/render/render_resource_state.hpp` is the tiny state wrapper that keeps an image and its current layout together.

It defines:

- `TrackedImage`: a non-owning view over `rhi::Image` plus a mutable layout reference
- `FrameTrackedImage`: an owning per-frame image with tracked layout
- `require_valid_tracked_image(...)`: basic validity check used by passes

This file exists because most render and present passes need both:

- the Vulkan image handle
- the current layout carried across barriers

Without this wrapper, layout tracking would be duplicated across every pass and backend.
