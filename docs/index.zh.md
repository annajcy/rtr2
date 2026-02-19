# RTR2

RTR2 是一个使用 C++23、Vulkan 和 Slang 着色器编写的现代实时三维渲染引擎。

## 特性亮点

- 基于 Vulkan RHI 的实时渲染框架
- 集成 ImGui 编辑器
- 通过 PBPT 接入离线路径追踪
- 包含单元测试和 GPU 集成测试套件

## 目录结构

- `src/rtr/`：引擎与运行时源码
- `shaders/`：Slang 着色器程序
- `examples/`：可运行的示例程序
- `test/`：基于 GoogleTest 的测试套件

前往 [快速上手](getting-started.md) 开始使用。

查看 [API 文档](api/index.md) 获取由 Doxygen 生成的 API 参考。
