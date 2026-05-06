# Scene Target Controller

`src/rtr/system/render/scene_target_controller.hpp` 负责管理像 `ForwardPipeline`、`ShaderToyPipeline` 这类 pipeline 的 resize-aware、per-frame 离屏目标。

它把这几件麻烦事集中处理掉了：

- 判断 scene viewport 或 swapchain extent 是否要求新的离屏尺寸
- 按需懒重建目标集合
- 在所有 frame slot 都安全之前延迟回收旧目标
- 暴露当前 generation 和本帧是否重建

controller 会监听 owner pipeline 发出的 `SceneViewportResizeEvent`，再和 renderer 路径传进来的 fallback extent 组合起来决定真正场景目标尺寸。

它的重要性在于：引擎里“场景渲染尺寸”和“窗口 framebuffer 尺寸”经常不是一回事，尤其是在 editor scene view 场景下。
