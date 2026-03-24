# ShaderToy Pipeline 总览

`src/rtr/system/render/pipeline/shadertoy/` 存放的是基于 compute 的全屏图像生成 pipeline。

当前包括：

- `shadertoy_compute_pass.hpp`：可复用的 compute pass
- `shadertoy_pipeline.hpp`：固定 shader 的 compute pipeline
- `editable_shadertoy_pipeline.hpp`：支持源码热重载的可编辑版本
- `editable_shadertoy_reload_controller.hpp`：热重载状态机和编译触发逻辑

这些 pipeline 完全在 GPU 上生成最终图像，再通过 `final_output(...)` 把结果交给 backend 去 present 或 compose。
