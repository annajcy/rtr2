# Slang Shader Compiler

这个目录包含了自动编译 Slang shader 文件到 SPIR-V 的工具。

## 构建流程

在CMake构建过程中，会自动执行以下步骤：

1. 首先构建 `compile_slang` 可执行文件
2. 然后运行 `compile_slang` 来递归编译所有 `.slang` 文件
3. 编译后的 SPIR-V 文件会输出到 `build/Debug/slang/compiled/` 目录

## 使用方法

### 自动编译（推荐）

在项目根目录运行 CMake 构建即可：

```bash
cd build/Debug
cmake ../..
cmake --build .
```

所有 `.slang` 文件会自动编译成 `.spv` 文件。

### 手动编译

也可以手动运行编译器：

```bash
cd build/Debug
./slang/compile_slang <slang_source_dir> [output_dir]
```

例如：
```bash
./slang/compile_slang ../../slang ./compiled_shaders
```

## 添加新的 Shader

只需在 `slang/` 目录（或其子目录）中添加 `.slang` 文件，下次构建时会自动编译。

## 输出

- 编译后的 SPIR-V 文件保存在 `build/Debug/slang/compiled/` 目录
- 保持原有的目录结构
- 文件扩展名从 `.slang` 改为 `.spv`

## 示例

如果有以下文件结构：
```
slang/
  ├── test.slang
  └── shaders/
      └── lighting.slang
```

编译后会生成：
```
build/Debug/slang/compiled/
  ├── test.spv
  └── shaders/
      └── lighting.spv
```
