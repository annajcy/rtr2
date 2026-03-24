# ShaderToy Pipeline Overview

`src/rtr/system/render/pipeline/shadertoy/` contains compute-driven full-screen image pipelines.

The directory currently contains:

- `shadertoy_compute_pass.hpp`: the reusable compute pass
- `shadertoy_pipeline.hpp`: fixed-shader compute pipeline
- `editable_shadertoy_pipeline.hpp`: editable variant with source-file reload
- `editable_shadertoy_reload_controller.hpp`: reload state machine and compile trigger logic

These pipelines generate their final image entirely on the GPU, then expose it through `final_output(...)` for the backend to present or compose.
