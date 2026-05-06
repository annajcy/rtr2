# Forward Pipeline Overview

`src/rtr/system/render/pipeline/forward/` contains the engine's forward scene-rendering path.

The split inside this directory is:

- `forward_scene_view.hpp`: CPU-side scene view types consumed by the renderer
- `forward_pass.hpp`: the actual graphics pass that renders meshes into offscreen color/depth targets
- `forward_pipeline.hpp`: pipeline orchestration, scene extraction, resource setup, and final-output exposure

The forward path is the main scene renderer used by realtime, editor, and current offline preview/export workflows.
