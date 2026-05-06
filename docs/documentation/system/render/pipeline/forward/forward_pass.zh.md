# Forward Pass

`src/rtr/system/render/pipeline/forward/forward_pass.hpp` 是真正录制 forward 场景 draw call 的底层 graphics pass。

它消费的资源包括：

- 一张 tracked 的离屏 color image
- 一张 depth image
- 当前 render extent
- 一组 `ForwardPassDrawItem`

这个 pass 负责：

- 把 color/depth 目标切到可渲染 layout
- 启动 dynamic rendering
- 绑定 forward graphics pipeline 和 per-object descriptor set
- 对每个 draw item 发出 indexed draw

它不负责构建 scene 数据，也不负责管理目标生命周期；这些职责仍在 `ForwardPipeline`。
