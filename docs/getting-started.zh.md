# 快速上手

## 前置依赖

- CMake 3.27+
- Conan 2+
- Ninja
- Python（需安装 `uv`）

## 构建（Debug 模式）

```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug
```

## 运行单元测试

```bash
ctest --test-dir build/Debug -C Debug -LE integration
```

## 运行 GPU 集成测试

```bash
RTR_RUN_GPU_TESTS=1 ctest --test-dir build/Debug -C Debug -L integration
```

## 构建文档

```bash
uv run mkdocs build
```
