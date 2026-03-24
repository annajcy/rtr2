# Forward Pipeline 总览

`src/rtr/system/render/pipeline/forward/` 是引擎当前的 forward 场景渲染路径。

这层目录内部的职责拆分是：

- `forward_scene_view.hpp`：渲染前消费的 CPU 侧场景视图数据
- `forward_pass.hpp`：真正把 mesh 画进离屏 color/depth 目标的 graphics pass
- `forward_pipeline.hpp`：scene 提取、资源准备、目标管理和最终输出暴露

这条 forward 路径是 realtime、editor，以及当前 offline preview/export 工作流共享的主场景渲染器。
