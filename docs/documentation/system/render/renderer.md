# Renderer

`src/rtr/system/render/renderer.hpp` contains the swapchain-backed renderer bootstrap used by realtime, editor, and preview-style offline paths.

The key layers are:

- `IRenderer`: small runtime-facing interface
- `RendererT<TBackend>`: templated renderer parameterized by an output backend
- `ComputeJob`: helper for one-off asynchronous compute work

`RendererT` currently owns:

- `rhi::Window`
- `rhi::Context`
- `rhi::Device`
- `FrameScheduler`
- one active `RenderPipeline`
- one backend instance

Its main responsibilities are:

- build the `PipelineRuntime`
- keep pipeline assignment immutable after binding
- synchronize swapchain state changes into the pipeline
- delegate final output handling to the selected backend

Current coupling:

- bootstrap is still window + surface + swapchain driven
- `draw_frame()` is only available for backends that satisfy the current swapchain-oriented contract
