# Editable ShaderToy Reload Controller

`src/rtr/system/render/pipeline/shadertoy/editable_shadertoy_reload_controller.hpp` 定义了一个状态机，用来决定何时重新编译可编辑 ShaderToy 程序。

它跟踪的状态包括：

- 请求的源码路径
- 是否开启 auto reload
- 是否有显式 reload 请求
- 上一次成功编译的路径和文件时间戳
- 对外暴露给 UI/runtime 的 reload 状态

这个 controller 本身不直接构建 pipeline。它只负责解析源码路径、比较时间戳，并记录编译成功/失败状态。
