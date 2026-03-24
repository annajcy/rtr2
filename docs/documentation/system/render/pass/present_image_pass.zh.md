# Present Image Pass

`src/rtr/system/render/pass/present_image_pass.hpp` 是另一种输出路径：通过全屏绘制来完成最终呈现，而不是直接 blit。

和 `PresentPass` 不同，它会：

- 通过 descriptor set 采样一张 offscreen image
- 绑定图形 pipeline
- 绘制一个全屏三角形到输出目标
- 同时为目标帧准备 depth attachment

这种方式适合需要 shader 驱动合成，而不是原始 transfer copy 的输出场景。

当前状态：

- 这个 pass 仍然面向 swapchain-backed 输出
- 目前主要作为 image-to-screen 合成的可复用工具保留
