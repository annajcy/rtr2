# Forward Scene View

`src/rtr/system/render/pipeline/forward/forward_scene_view.hpp` defines the CPU-side data model that the forward renderer consumes.

It includes:

- `MeshView`: raw GPU buffer handles plus index count
- `ForwardSceneCameraData`: view/projection matrices and camera world position
- `ForwardScenePointLight`: compact per-light data
- `ForwardSceneRenderable`: one drawable mesh instance plus transforms and base color
- `ForwardSceneView`: the full scene payload sent to the forward path

This file is the boundary between scene extraction and GPU recording. Builders under `framework/integration/render/` produce this view; `ForwardPipeline` consumes it.
