# Render Pipelines Overview

`src/rtr/system/render/pipeline/` contains the pure content pipelines that feed the renderer's output backends.

The current pipeline families are:

- `forward/`: graphics-scene rendering with meshes, lights, and camera extraction
- `shadertoy/`: compute-driven full-screen image generation, including editable hot-reload variants

All pipeline families share the same base contract from `render_pipeline.hpp`:

- render into offscreen targets
- expose `final_output(...)`
- let backends handle present, editor composition, or export
