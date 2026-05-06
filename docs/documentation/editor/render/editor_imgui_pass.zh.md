# Editor ImGui Pass

`src/rtr/editor/render/editor_imgui_pass.hpp` 负责录制实际的 ImGui / editor 绘制 pass。

它的工作包括：

- 管理 `rhi::ImGuiContext`
- 维护逐帧 scene texture descriptor 的生命周期
- 向 editor 暴露 scene texture ID、viewport rect、hover/focus 状态和 gizmo 使用状态
- 把 ImGui draw data 画到当前输出目标

这个 pass 由 `EditorOutputBackend` 喂入：

- 可采样的场景图像 view/layout
- 最终输出目标
- 回传给内容 pipeline 的 scene viewport resize callback

当前耦合点：

- 它和 ImGui Vulkan descriptor 生命周期强绑定
- swapchain recreate 目前会强制刷新 scene texture descriptor
