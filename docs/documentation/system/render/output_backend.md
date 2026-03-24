# Output Backend

`src/rtr/system/render/output_backend.hpp` defines how a pipeline's final offscreen image is consumed after scene rendering finishes.

Current contract:

- backends implement `begin_frame()`
- pipelines render into a `FrameContext`
- backends consume `pipeline.final_output(...)` in `record_output(...)`
- backends finish submission and presentation/export in `end_frame(...)`

Important types:

- `RenderFrameTicket`: backend-owned per-frame record token
- `RenderOutputBackendConcept`: compile-time contract used by `RendererT`
- `RenderBackendServices`: device/context/window/frame-scheduler services shared with a backend
- `SwapchainFrameOutputBackendBase`: helper base for swapchain-backed paths

Concrete backends in this file:

- `SwapchainOutputBackend`: present the final offscreen image to the swapchain
- `OfflineImageOutputBackend`: optional PNG export while still using the swapchain-backed preview path

Current coupling:

- `RenderBackendServices` still includes `rhi::Window&` and `FrameScheduler&`
- `OfflineImageOutputBackend` is preview-capable, not a true no-surface headless backend
