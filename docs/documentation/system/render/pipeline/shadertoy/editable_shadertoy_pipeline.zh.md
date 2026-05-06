# Editable ShaderToy Pipeline

`src/rtr/system/render/pipeline/shadertoy/editable_shadertoy_pipeline.hpp` 在 ShaderToy 路径上加入了基于源码文件的 shader 热重载能力。

它整体仍然沿用 `ShaderToyPipeline` 的渲染结构，但额外增加了：

- reload controller
- 可选 auto-reload
- 显式 reload 请求
- 当前 program 无效时的回退行为

编译成功时，它会重建 compute shader module 和 pipeline；编译失败时，pipeline 仍然保持为一个可控制的对象，只是输出退化为黑屏。
