# Render Resource State

`src/rtr/system/render/render_resource_state.hpp` 是一个很小但很关键的状态封装：把 image 和它当前的 layout 绑在一起。

它定义了：

- `TrackedImage`：对 `rhi::Image` 和可变 layout 引用的非拥有视图
- `FrameTrackedImage`：拥有每帧 image 与 layout 的封装
- `require_valid_tracked_image(...)`：供 pass 复用的基础校验

这个文件存在的原因是，大多数 render/present pass 都同时需要：

- Vulkan image handle
- 跨 barrier 持续跟踪的当前 layout

如果没有这个封装，每个 pass 和 backend 都会重复维护一套 layout 状态。
