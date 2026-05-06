# Shader Compiler

`src/rtr/system/render/utils/shader_compiler.hpp` 把基于文件的 Slang 编译封装成了一个可供 runtime 直接消费的小结果对象。

它提供：

- `SlangFileCompileResult`
- `compile_slang_file_to_spirv(...)`

这个辅助函数会完成：

- 源文件存在性和时间戳检查
- Slang session/request 的构建
- 请求 stage 对应 entry point 的校验
- 把 SPIR-V 提取到内存字节数组

它是一个面向 runtime 的工具，主要服务于热重载和 editable pipeline，而不是整个项目的离线 shader 构建链。
