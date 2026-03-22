# Phase 0: Eigen 引入

## 为什么 Day 1 第一件事是这个

当前仓库没有 Eigen 依赖。IPC solver 内部统一用 Eigen，不在核心路径混 `pbpt::math`。如果 Eigen 接不进来，后面所有代码都写不了。

## 具体步骤

### 1. conanfile.py 加依赖

在 `requirements()` 中加：

```python
self.requires("eigen/3.4.0", transitive_headers=True)
```

### 2. src/CMakeLists.txt 加链接

`rtr_runtime` 的 `target_link_libraries` 加：

```cmake
Eigen3::Eigen
```

注意：Conan 2 的 Eigen 包导出的 target 名是 `Eigen3::Eigen`。

### 3. test/CMakeLists.txt

`rtr_add_test` 函数已经 include `${CMAKE_SOURCE_DIR}/src`，所以 IPC 测试只要 link `Eigen3::Eigen` 即可。后续 IPC 测试注册时加这个依赖。

### 4. 验证

写一个最小文件确认编译通过：

```cpp
#include <Eigen/Dense>
#include <Eigen/Sparse>

static_assert(sizeof(Eigen::VectorXd) > 0);
```

### 5. 重新构建

```bash
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug
```

## 验收

- `cmake --build` 成功
- 现有测试不受影响
- 能在新文件中 `#include <Eigen/Dense>` 编译通过

## 风险

- Conan 缓存可能有 Eigen 旧版本冲突 -> `conan remove 'eigen/*' -c` 后重试
- Eigen 和某些编译器 C++23 特性有已知 warning -> 先忽略，不影响正确性
