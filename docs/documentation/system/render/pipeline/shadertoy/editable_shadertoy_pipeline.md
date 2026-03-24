# Editable ShaderToy Pipeline

`src/rtr/system/render/pipeline/shadertoy/editable_shadertoy_pipeline.hpp` extends the ShaderToy path with source-file driven shader reload.

It keeps the same broad rendering structure as `ShaderToyPipeline`, but adds:

- a reload controller
- optional auto-reload
- explicit reload requests
- fallback behavior when the current program is invalid

On a successful compile, it rebuilds the compute shader module and pipeline. On failure, it keeps the pipeline alive as a controllable editor/runtime object while rendering a black fallback output.
