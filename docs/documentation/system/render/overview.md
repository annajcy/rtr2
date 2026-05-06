# Render System Overview

`src/rtr/system/render/` is the engine's real-time GPU render stack.

The current tree is organized by responsibility:

- root files: frame orchestration, pipeline contracts, backend abstraction, and per-frame state
- `pass/`: reusable output-oriented render passes such as present passes
- `pipeline/forward/`: scene rendering through the forward graphics path
- `pipeline/shadertoy/`: compute-driven full-screen pipelines and hot-reload support
- `utils/`: shader compilation helpers

The key architectural split in the current implementation is:

- pipelines render into a final offscreen image
- output backends decide how that image is consumed
- the renderer owns window/context/device/swapchain bootstrap for the realtime and preview paths

See the file-level pages under this subtree for the concrete types and lifecycle details.
