# Forward Pipeline

`src/rtr/system/render/pipeline/forward/forward_pipeline.hpp` 是包裹 forward graphics pass 的高层场景渲染 pipeline。

它的职责包括：

- 加载 forward vertex/fragment shader
- 构建 descriptor layout、descriptor pool 和 graphics pipeline 状态
- 把活动场景提取成 `ForwardSceneView`
- 通过 `SceneTargetController` 维护每帧离屏 color/depth 目标
- 通过 `final_output(...)` 暴露当前 offscreen color image

内部最重要的职责拆分是：

- `prepare_frame(...)`：从 active world/scene 提取场景数据
- `render(...)`：确保目标存在、录制 `ForwardPass`，并把结果切到可采样 layout

这个 pipeline 现在是纯内容渲染器，不再拥有 present。realtime、editor 和 preview/export 的差异都来自所选 backend。
