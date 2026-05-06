# Frame Context

`src/rtr/system/render/frame_context.hpp` 定义了传给每个 render pass 和 pipeline 的逐帧录制上下文。

它把录制一帧时最小但关键的状态绑在一起：

- 当前 `rhi::CommandBuffer`
- 当前 `rhi::Device`
- 渲染 extent
- 当前 frame slot index
- 可选的 `RenderOutputTarget`

它是纯内容渲染和 output backend 之间的桥。pipeline 总是可以往离屏图像里渲染，而 backend 持有的 pass 只有在存在 swapchain 风格目标时才会访问 `output_target()`。

当前耦合点：

- 为兼容旧代码，仍保留了 `swapchain_image()` 和 `swapchain_image_view()`
- 调用方必须先检查 `has_output_target()`，不能默认本帧一定可 present
