# Render Passes Overview

`src/rtr/system/render/pass/` contains reusable output-oriented passes that sit after scene generation.

Current contents:

- `present_pass.hpp`: blit the pipeline's final offscreen image into a swapchain image
- `present_image_pass.hpp`: draw a sampled full-screen image into the swapchain through a graphics pipeline

These passes are backend-facing utilities. They are not content pipelines by themselves.
