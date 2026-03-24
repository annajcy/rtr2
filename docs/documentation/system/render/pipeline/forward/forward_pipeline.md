# Forward Pipeline

`src/rtr/system/render/pipeline/forward/forward_pipeline.hpp` is the high-level scene-render pipeline that wraps the forward graphics pass.

Its responsibilities include:

- loading forward vertex/fragment shaders
- building descriptor layouts, descriptor pools, and graphics pipeline state
- extracting the active scene into `ForwardSceneView`
- maintaining per-frame offscreen color/depth targets through `SceneTargetController`
- exposing the current offscreen color image through `final_output(...)`

Important internal split:

- `prepare_frame(...)` extracts scene data from the active world/scene
- `render(...)` ensures targets, records `ForwardPass`, and transitions the result into a sampled layout

This pipeline is pure content rendering. It no longer owns present; realtime, editor, and preview/export behavior comes from the chosen backend.
