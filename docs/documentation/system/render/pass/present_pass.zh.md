# Present Pass

`src/rtr/system/render/pass/present_pass.hpp` 定义了 swapchain 路径上使用的基于 blit 的 present pass。

它做三件事：

- 把源 offscreen color image 切到 `eTransferSrcOptimal`
- 把目标输出图像切到 `eTransferDstOptimal`
- 执行 blit，并把两边恢复到后续阶段需要的 layout

这个 pass 默认 `FrameContext::output_target()` 提供的是 swapchain 风格目标。实际使用者是 realtime present 和带 preview 能力的 offline output。

当前耦合点：

- 实现里仍通过 `ctx.swapchain_image()` 这类兼容接口访问目标
- 它只解决 present-style 输出，不负责通用图片导出
