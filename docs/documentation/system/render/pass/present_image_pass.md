# Present Image Pass

`src/rtr/system/render/pass/present_image_pass.hpp` is the fullscreen draw variant of output presentation.

Unlike `PresentPass`, which blits, this pass:

- samples an offscreen image through a descriptor set
- binds a graphics pipeline
- draws a fullscreen triangle into the output target
- also prepares a depth attachment for the destination frame

This style is useful when presentation needs shader-driven composition rather than a raw transfer copy.

Current status:

- the pass still targets swapchain-backed output
- it survives mainly as a reusable utility for image-to-screen composition code paths
