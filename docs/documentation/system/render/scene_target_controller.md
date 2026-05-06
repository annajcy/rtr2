# Scene Target Controller

`src/rtr/system/render/scene_target_controller.hpp` manages resize-aware, per-frame offscreen targets for pipelines such as `ForwardPipeline` and `ShaderToyPipeline`.

Its job is to hide the awkward parts of scene target lifetime:

- detect when the scene viewport or swapchain extent implies a new offscreen size
- lazily recreate the target set
- keep old targets alive until every frame slot is fence-safe
- expose the current generation and recreate status

The controller listens to `SceneViewportResizeEvent` from its owning pipeline and combines that with a fallback extent from the renderer path.

This class is important because the engine frequently needs "scene render size" to diverge from "window framebuffer size", especially in editor scene views.
