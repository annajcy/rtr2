# Present Pass

`src/rtr/system/render/pass/present_pass.hpp` defines the blit-based present path used by swapchain-backed output backends.

It performs three pieces of work:

- transition the source offscreen color image into `eTransferSrcOptimal`
- transition the destination output target into `eTransferDstOptimal`
- blit the source into the destination and restore layouts for downstream use

The pass assumes a swapchain-like destination is available through `FrameContext::output_target()`. In practice it is used by realtime present and preview-capable offline output.

Current coupling:

- the implementation still uses compatibility accessors such as `ctx.swapchain_image()`
- it is specifically about present-style output, not generic image export
