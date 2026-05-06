# ShaderToy Pipeline

`src/rtr/system/render/pipeline/shadertoy/shadertoy_pipeline.hpp` 是固定程序版本的全屏过程化 compute pipeline。

它的职责包括：

- 加载一个 compute shader module
- 构建 descriptor layout、pool 和 compute pipeline 状态
- 为每个 frame slot 分配一张离屏图像
- 在目标 generation 变化时刷新 storage-image descriptor
- 执行 `ComputePass` 并暴露最终可采样图像

和 `ForwardPipeline` 不同，这个 pipeline 不消费 world 中的场景数据。它的主要动态输入是时间、输出尺寸和用户参数。
