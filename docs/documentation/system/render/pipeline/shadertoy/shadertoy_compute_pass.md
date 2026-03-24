# ShaderToy Compute Pass

`src/rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp` records the compute work shared by ShaderToy-style pipelines.

It is responsible for:

- writing `iResolution`, `iTime`, and custom params into a mapped uniform buffer
- transitioning the offscreen image into `eGeneral`
- binding the compute pipeline and descriptor set
- dispatching workgroups over the full output image

The pass does not own shader modules, descriptor allocation, or target lifetime. Those remain in the surrounding pipeline class.
