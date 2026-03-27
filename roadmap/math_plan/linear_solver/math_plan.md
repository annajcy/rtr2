# pbpt::math IPC 高性能线性代数扩展计划

## 目标

基于现有 `pbpt::math` 库，扩展出一套适用于 IPC 物理求解的高性能线性代数模块，**完全替换 Eigen3 依赖**。

核心原则：

- **务实渐进**：分层替换，每层独立可验证可回退
- **CPU 先行**：先做 SIMD 友好的 CPU 高性能版本，架构上预留 GPU 扩展接口
- **零开销抽象**：编译期策略模式，不引入虚函数开销
- **风格一致**：沿用 `pbpt::math` 的 header-only、constexpr、C++20 concepts 风格

## 背景：当前 Eigen3 在 IPC 中的使用

当前 IPC 系统对 Eigen 的依赖分布在以下几个层面：

| 层面 | Eigen 功能 | 涉及文件 |
|------|-----------|---------|
| 状态存储 | `VectorXd` 动态向量 | `ipc_state.hpp`, 各 energy 模块 |
| 密集运算 | `Matrix3d` 运算、`dot`、`cross`、`cwiseProduct` | 全部 IPC 模块 |
| 矩阵分解 | `JacobiSVD<Matrix3d>` 极分解 | `tet_fixed_corotated.hpp` |
| 自动微分 | `AutoDiffScalar` 二阶 | `distance_*.hpp` |
| 稀疏存储 | `SparseMatrix<double>`, `Triplet<double>` | `newton_solver.hpp` |
| 稀疏求解 | `SimplicialLDLT` | `newton_solver.hpp` |

其中 `pbpt::math` 已有的固定尺寸 `Matrix<T,R,C>` 和 `Vector<T,N>` 可以直接替换密集运算部分。需要新增的是：动态向量、稀疏矩阵、SVD 分解、求解器接口。

## 新增模块架构

```
pbpt/math/
├── basic/          # 已有
├── spatial/        # 已有
├── matrix/         # 已有
├── complex/        # 已有
├── special/        # 已有
│
├── dynamic/        # 【新增】动态尺寸线性代数
│   ├── dvector.hpp
│   └── dynamic.h
│
├── sparse/         # 【新增】稀疏矩阵
│   ├── triplet.hpp
│   ├── sparse_matrix.hpp
│   ├── sparse_builder.hpp
│   └── sparse.h
│
├── decomposition/  # 【新增】矩阵分解
│   ├── svd3x3.hpp
│   ├── polar3x3.hpp
│   └── decomposition.h
│
├── solver/         # 【新增】线性求解器
│   ├── linear_solver.hpp
│   ├── backend/
│   │   ├── suitesparse_ldlt.hpp
│   │   ├── eigen_ldlt.hpp        # 过渡后端
│   │   └── mkl_pardiso.hpp       # 可选
│   └── solver.h
│
├── autodiff/       # 【预留】自动微分（后期开发）
│
└── math.h
```

## 分层替换策略

整个替换分 4 层执行，每层完成后系统均可编译运行：

- **Layer 0** — 动态向量 `DVector<T>`（替换 `Eigen::VectorXd`）
- **Layer 1** — 3x3 SVD + 解析距离导数（替换 `JacobiSVD` 和 `AutoDiff`）
- **Layer 2** — 稀疏矩阵 + 求解器后端（替换 `SparseMatrix` 和 `SimplicialLDLT`）
- **Layer 3** — 完全移除 Eigen 依赖

详见各 Layer 文档。

## GPU 扩展预留点

不在本期实现，但架构上已预留：

- `DVector` 的 Allocator 模板参数 → 未来接 CUDA/Metal device allocator
- `SparseMatrix` 暴露 CSR 原始指针 → cuSPARSE 直接消费
- `LinearSolver` 的 Backend 模板参数 → 未来加 `CuSolverBackend`
- `svd3x3` 的纯算术实现 → 可直接移植为 GPU kernel

## 长期目标

1. **自研稀疏求解器**：在 SuiteSparse 后端稳定后，逐步开发自研 LDLT/Cholesky
2. **自动微分模块**：在解析导数覆盖当前需求后，按需开发通用 `autodiff/` 模块
3. **通用 SVD**：在 3x3 专用版稳定后，扩展到任意尺寸 Jacobi SVD
4. **GPU 求解管线**：CSR + cuSolver/cuSPARSE 全 GPU 路径
