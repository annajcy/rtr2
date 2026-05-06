# ShaderToy Compute Pass

`src/rtr/system/render/pipeline/shadertoy/shadertoy_compute_pass.hpp` 负责录制 ShaderToy 风格 pipeline 共享的 compute 工作。

它的职责是：

- 把 `iResolution`、`iTime` 和自定义参数写进映射的 uniform buffer
- 把离屏图像切到 `eGeneral`
- 绑定 compute pipeline 和 descriptor set
- 按整张输出图像 dispatch workgroup

这个 pass 不拥有 shader module、descriptor 分配或目标生命周期；这些仍在外围 pipeline 类里管理。
