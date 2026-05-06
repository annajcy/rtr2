# IPC Solver 总览

`solver/` 目录放的是当前 IPC 的非线性求解工具。

## 当前文件

- `line_search.hpp`
  沿给定搜索方向做 Armijo backtracking line search。
- `newton_solver.hpp`
  一个最小可用的 Newton 求解器，工作在全局 energy / gradient / Hessian 回调之上，支持 per-DOF Dirichlet mask，并使用 sparse reduced solve。

## 当前范围

这一层目前负责：

- 全局 DOF 空间中的 Newton 步
- line search
- 收敛判定
- 在 per-DOF fixed mask 下的 reduced solve

目前还不包含：

- PSD projection
- CCD 步长裁剪
- contact-aware filtering
- friction solve orchestration
