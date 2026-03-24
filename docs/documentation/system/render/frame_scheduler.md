# Frame Scheduler

`src/rtr/system/render/frame_scheduler.hpp` is the swapchain-backed frame orchestration layer for realtime and preview rendering.

Its responsibilities are:

- own the swapchain
- own per-frame command buffers, semaphores, and fences
- acquire the next swapchain image
- submit recorded work and present it
- recreate swapchain resources after resize or out-of-date events

The main data types are:

- `PerFrameResources`: command buffer + acquire semaphore + in-flight fence
- `PerImageResources`: render-finished semaphore
- `FrameTicket`: frame slot, image slot, and command buffer for the current frame
- `SwapchainState`: generation, extent, image count, and formats

This class is intentionally presentation-specific. Current headless planning treats it as a realtime/preview utility, not a general frame executor abstraction.
