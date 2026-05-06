# ShaderToy Pipeline

`src/rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp` is the fixed-program compute pipeline for full-screen procedural rendering.

Its responsibilities are:

- load one compute shader module
- build descriptor layout, pool, and compute pipeline state
- allocate one offscreen image per frame slot
- refresh storage-image descriptors when target generations change
- run `ComputePass` and expose the sampled final image

Compared with `ForwardPipeline`, this pipeline does not consume scene data from the world. Its primary dynamic inputs are time, output extent, and user-controlled params.
