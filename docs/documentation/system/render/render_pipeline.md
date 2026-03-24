# Render Pipeline

`src/rtr/system/render/render_pipeline.hpp` defines the base contract shared by all pure content pipelines.

The current contract has three important parts:

- `render(FrameContext&)`: record the pipeline's GPU work
- `final_output(frame_index)`: expose the final offscreen image for a backend to consume
- `prepare_frame(FramePrepareContext)`: update pipeline-side scene data before recording

Supporting types:

- `PipelineRuntime`: construction-time services such as device, context, formats, and shader root path
- `FramePrepareContext`: per-frame world/resource/input state used before rendering
- `SwapchainChangeSummary`: diff object for swapchain-dependent rebuild logic
- `SceneViewportResizeEvent`: event used by scene-target-aware pipelines

Current coupling:

- `PipelineRuntime` still carries `rhi::Window&`
- the base class still tracks swapchain-derived extent and formats for pipeline rebuild logic

Even with that coupling, the ownership split is already explicit: pipelines produce the final image; output backends decide how it is presented or exported.
