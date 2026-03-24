# Render Pipelines 总览

`src/rtr/system/render/pipeline/` 存放的是纯内容 pipeline，它们为 renderer 的 output backend 提供最终图像。

当前主要有两类：

- `forward/`：基于 graphics 的场景渲染，包含 mesh、light 和 camera 提取
- `shadertoy/`：基于 compute 的全屏图像生成，以及可编辑的热重载变体

所有 pipeline 家族都遵循 `render_pipeline.hpp` 的同一套基类契约：

- 渲染到离屏目标
- 暴露 `final_output(...)`
- 把 present、editor compose 或 export 留给 backend
