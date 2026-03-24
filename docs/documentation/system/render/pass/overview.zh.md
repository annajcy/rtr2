# Render Passes 总览

`src/rtr/system/render/pass/` 存放的是可复用的输出型 pass，它们位于场景内容生成之后。

当前包括：

- `present_pass.hpp`：把 pipeline 的最终 offscreen image blit 到 swapchain image
- `present_image_pass.hpp`：通过图形 pipeline 把一张采样图像画到 swapchain 上

这些 pass 是 backend 侧工具，不是独立的内容 pipeline。
