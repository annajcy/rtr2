# Layer 3 — 完全移除 Eigen 依赖

## 目标

在 Layer 0-2 完成后，清除项目中所有 Eigen 的残留引用，确保 IPC 系统完全运行在 `pbpt::math` 之上。

## 预计工期：1-2 天

## 步骤

### 1. 全局搜索残留 Eigen 引用

```bash
# 搜索所有 Eigen include
grep -rn "#include.*Eigen" src/rtr/system/physics/ipc/
grep -rn "#include.*Eigen" external/pbpt/src/pbpt/math/

# 搜索 Eigen 命名空间引用
grep -rn "Eigen::" src/rtr/system/physics/ipc/
```

预期 Layer 2 完成后，唯一残留应该是 `EigenBackend` 内部。

### 2. 删除 EigenBackend

- [ ] 删除 `external/pbpt/src/pbpt/math/solver/backend/eigen_ldlt.hpp`
- [ ] 确认无代码引用此文件

### 3. 清理 CMakeLists.txt

- [ ] 移除 `find_package(Eigen3)` 或将其设为 OPTIONAL（如果项目其他部分仍用 Eigen）
- [ ] 移除 IPC 模块对 Eigen 的 `target_link_libraries`
- [ ] 如果全项目不再使用 Eigen，移除 Eigen 的 submodule / fetch 配置

### 4. 清理 Newton Solver

- [ ] 移除 `newton_solver.hpp` 中任何遗留的 Eigen 头文件和临时转换工具
- [ ] 移除 Layer 0 中添加的 `to_eigen_map()` 临时互操作函数

### 5. 更新 umbrella headers

- [ ] 运行 `generate_pbpt_header.py` 更新 `math.h` 入口（包含新增的 dynamic, sparse, decomposition, solver 模块）
- [ ] 或手动更新 `math.h` 的 include 列表

## 验证方案

### 编译验证

```bash
# 确认无 Eigen 头文件被包含
grep -rn "#include.*Eigen" src/rtr/system/physics/ipc/
# 期望输出为空

# 全量编译
cmake --build build --target rtr2
# 期望无 Eigen 相关编译错误
```

### 运行验证

- IPC 全部测试场景通过
- 对比 Layer 2 结果，数值完全一致（因为只删除了未使用的代码）

### 依赖验证

```bash
# 确认编译产物不链接 Eigen
# (Eigen 是 header-only，所以主要确认头文件不再被包含)
```

## 后续清理（可选）

- 如果项目其他模块（非 IPC）也使用 Eigen，可以逐步迁移
- 如果决定全项目去 Eigen，在此处记录剩余依赖点
