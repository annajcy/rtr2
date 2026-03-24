# Forward Pass

`src/rtr/system/render/pipeline/forward/forward_pass.hpp` is the low-level graphics pass that records the actual forward scene draw calls.

Its input resources are:

- a tracked offscreen color image
- a depth image
- the render extent
- a list of `ForwardPassDrawItem`

The pass is responsible for:

- transitioning color and depth targets into renderable layouts
- starting dynamic rendering
- binding the forward graphics pipeline and per-object descriptor sets
- issuing indexed draws for every draw item

It does not build scene data or own target lifetime. Those responsibilities stay in `ForwardPipeline`.
