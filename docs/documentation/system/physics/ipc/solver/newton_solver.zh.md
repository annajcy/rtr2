# `newton_solver.hpp`

`src/rtr/system/physics/ipc/solver/newton_solver.hpp` 实现了 IPC 优化问题的 Newton 求解器。

## 背景：为什么用 Newton 法

IPC 的一个时间步被表述为无约束优化：

$$
x^{k+1} = \arg\min_x E(x)
$$

其中 $E(x) = E_{\text{inertial}}(x) + E_{\text{gravity}}(x) + E_{\text{elastic}}(x) + \ldots$

Newton 法通过反复求解局部二次近似来寻找极小值。

### 二次近似

在第 $k$ 次迭代，用二阶 Taylor 展开逼近 $E$：

$$
E(x^k + \Delta x) \approx E(x^k) + g^T \Delta x + \frac{1}{2} \Delta x^T H \Delta x
$$

其中 $g = \nabla E(x^k)$ 是梯度，$H = \nabla^2 E(x^k)$ 是 Hessian。

令 $\nabla_{\Delta x} = 0$：

$$
g + H \Delta x = 0
$$

$$
\boxed{H \Delta x = -g}
$$

这就是 **Newton 方程**。解出的 $\Delta x$ 是局部二次模型的极小化步。

### 为什么 Newton 法收敛快

**定理（二次收敛）**：若 $H$ 在极小点 $x^*$ 附近 Lipschitz 连续且正定，且 $x^0$ 足够近，则：

$$
\|x^{k+1} - x^*\| \le C \|x^k - x^*\|^2
$$

这意味着正确有效位数每次迭代大约翻倍。对 IPC 而言，每个时间步通常需要 5-20 次迭代收敛。

### Newton 与梯度下降对比

| 特性 | Newton | 梯度下降 |
|------|--------|---------|
| 步 | $\Delta x = -H^{-1} g$ | $\Delta x = -g$ |
| 收敛 | 二次 | 线性 |
| 单次迭代代价 | $O(n)$ 稀疏求解 | $O(n)$ 向量运算 |
| 刚度敏感性 | 低（Hessian 自适应） | 高（需要小步长） |

对于刚性弹性问题（大杨氏模量），梯度下降需要极小步长。Newton 法天然适应刚度，因为 Hessian 编码了局部曲率。

## 数据结构

### `NewtonSolverParams`

```cpp
struct NewtonSolverParams {
    int max_iterations{50};        // 每步最大 Newton 迭代次数
    double gradient_tolerance{1e-6}; // ||g||_inf 收敛阈值
    double dx_tolerance{1e-8};      // ||Δx||_inf 收敛阈值
    double regularization{1e-8};    // 对角正则化 ε
    bool use_psd_projection{false}; // 预留给未来 PSD 投影
};
```

### `NewtonSolverResult`

```cpp
struct NewtonSolverResult {
    int iterations{0};              // 实际迭代次数
    double final_gradient_norm{0.0}; // 终止时的 ||g||_inf
    double final_energy{0.0};       // 终止时的 E(x)
    bool converged{false};          // 是否满足收敛条件
};
```

### `NewtonProblem`

```cpp
struct NewtonProblem {
    std::function<double(const Eigen::VectorXd& x)> compute_energy;
    std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd& gradient)> compute_gradient;
    std::function<void(const Eigen::VectorXd& x,
                       std::vector<Eigen::Triplet<double>>& triplets)> compute_hessian_triplets;
};
```

求解器通过回调与具体能量模型解耦。`IPCSystem` 将其 `compute_total_energy/gradient/hessian` 方法绑定到这些回调上。

## Dirichlet 边界条件：Reduced Solve

### 数学表述

给定自由 DOF 集 $\mathcal{F}$ 和约束 DOF 集 $\mathcal{C}$（$\mathcal{F} \cup \mathcal{C} = \{0, \ldots, 3n-1\}$），约束 Newton 步要求对 $i \in \mathcal{C}$ 有 $\Delta x_i = 0$。

分块：

$$
\begin{bmatrix} H_{FF} & H_{FC} \\ H_{CF} & H_{CC} \end{bmatrix}
\begin{bmatrix} \Delta x_F \\ 0 \end{bmatrix}
= -\begin{bmatrix} g_F \\ g_C \end{bmatrix}
$$

第一行给出：

$$
\boxed{H_{FF} \Delta x_F = -g_F}
$$

这就是 **reduced system**——只涉及 Hessian 的 free-free 子块和梯度的自由分量。

### 实现：索引重映射

**第 1 步**：收集自由 DOF 索引。

```cpp
inline std::vector<Eigen::Index> collect_free_dofs(const std::vector<bool>& free_dof_mask) {
    std::vector<Eigen::Index> free_dofs{};
    for (std::size_t i = 0; i < free_dof_mask.size(); ++i) {
        if (free_dof_mask[i]) {
            free_dofs.push_back(static_cast<Eigen::Index>(i));
        }
    }
    return free_dofs;
}
```

若全局 DOF 总数为 $3n$，其中 $m$ 个自由，`free_dofs` 有 $m$ 个元素。

**第 2 步**：将 fixed DOF 的梯度清零。

```cpp
inline void zero_fixed_gradient(Eigen::VectorXd& gradient, const std::vector<bool>& free_dof_mask) {
    for (Eigen::Index i = 0; i < gradient.size(); ++i) {
        if (!free_dof_mask[static_cast<std::size_t>(i)]) {
            gradient[i] = 0.0;
        }
    }
}
```

确保完整梯度中 $g_C = 0$，使得 masked infinity norm 只反映自由 DOF。

**第 3 步**：构建 reduced Hessian。

```cpp
inline Eigen::SparseMatrix<double> build_reduced_hessian(
    Eigen::Index dof_count,
    const std::vector<Eigen::Triplet<double>>& full_triplets,
    const std::vector<Eigen::Index>& free_dofs,
    double regularization)
```

过程：

1. 构建映射 `reduced_index[global_i]` → reduced 行/列索引（fixed DOF 映射为 $-1$）。

```cpp
std::vector<Eigen::Index> reduced_index(dof_count, -1);
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_index[free_dofs[i]] = i;
}
```

2. 过滤 triplets：只保留 `(i, j, v)` 中 `i` 和 `j` 都是自由的，重映射到 reduced 索引。

```cpp
for (const auto& triplet : full_triplets) {
    const Eigen::Index reduced_row = reduced_index[triplet.row()];
    const Eigen::Index reduced_col = reduced_index[triplet.col()];
    if (reduced_row < 0 || reduced_col < 0) continue;  // 涉及 fixed DOF
    reduced_triplets.emplace_back(reduced_row, reduced_col, triplet.value());
}
```

3. 添加对角正则化 $\varepsilon I$：

```cpp
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_triplets.emplace_back(i, i, regularization);
}
```

4. 组装稀疏矩阵，重复项累加：

```cpp
reduced_hessian.setFromTriplets(reduced_triplets.begin(), reduced_triplets.end(),
    [](const double lhs, const double rhs) { return lhs + rhs; });
```

自定义累加器确保重复条目（来自重叠的单元刚度矩阵）被累加而非覆盖。

**第 4 步**：提取 reduced gradient。

```cpp
Eigen::VectorXd reduced_gradient(free_dofs.size());
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_gradient[i] = gradient[free_dofs[i]];
}
```

**第 5 步**：求解 reduced system。

```cpp
inline bool solve_reduced_system(const Eigen::SparseMatrix<double>& reduced_hessian,
                                 const Eigen::VectorXd& reduced_gradient,
                                 Eigen::VectorXd& reduced_dx) {
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver{};
    solver.compute(reduced_hessian);
    if (solver.info() != Eigen::Success) return false;

    reduced_dx = solver.solve(-reduced_gradient);
    if (solver.info() != Eigen::Success || !reduced_dx.allFinite()) return false;
    return true;
}
```

`SimplicialLDLT` 执行 $LDL^T$ 分解（类似 Cholesky 但适用于对称矩阵，不要求正定——$D$ 可以有负元素）。复杂度：对有界带宽的稀疏矩阵为 $O(m)$。

**第 6 步**：展开回全局 DOF 空间。

```cpp
inline Eigen::VectorXd expand_reduced_step(Eigen::Index dof_count,
                                           const std::vector<Eigen::Index>& free_dofs,
                                           const Eigen::VectorXd& reduced_dx) {
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(dof_count);
    for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
        dx[free_dofs[i]] = reduced_dx[i];
    }
    return dx;
}
```

Fixed DOF 得到 $\Delta x_i = 0$（来自 `Zero` 初始化）。

## 收敛判据

求解器使用三个收敛判据，在不同时刻检查：

### 1. 梯度范数收敛

$$
\|g\|_\infty = \max_{i \in \mathcal{F}} |g_i| \le \varepsilon_g
$$

```cpp
const double gradient_norm = masked_inf_norm(gradient, free_dof_mask);
if (gradient_norm <= params.gradient_tolerance) {
    result.converged = true;
    return result;
}
```

**物理含义**：在极小值处 $\nabla E = 0$。无穷范数 $\|g\|_\infty$ 度量任意单个 DOF 上的最大力残差。当它低于阈值时，每个顶点都近似处于平衡状态。

**为什么用无穷范数**：与 $\|g\|_2$（随 $\sqrt{n}$ 增长，顶点越多范数越大）不同，$\|g\|_\infty$ 独立于网格分辨率。

### 2. 步长收敛

$$
\|\Delta x\|_\infty = \max_{i \in \mathcal{F}} |\Delta x_i| \le \varepsilon_{dx}
$$

```cpp
const double dx_norm = masked_inf_norm(dx, free_dof_mask);
if (dx_norm <= params.dx_tolerance) {
    result.converged = true;
    return result;
}
```

**物理含义**：Newton 步极小——位置几乎不变。

### 3. 更新后检查

更新 $x$ 后重新评估梯度：

```cpp
const double post_gradient_norm = evaluate_gradient_norm(problem, state.x, free_dof_mask);
if (post_gradient_norm <= params.gradient_tolerance ||
    line_search_result.alpha * dx_norm <= params.dx_tolerance) {
    result.converged = true;
    return result;
}
```

这能捕获 $\alpha < 1$ 使实际位移（$\alpha \cdot \Delta x$）足够小的情况。

## 正则化策略

### 为什么需要正则化

Hessian 可能不定，当：
- 能量函数非凸（例如固定共旋在压缩下）
- 数值误差累积
- 尚未实现 PSD 投影

不定 Hessian 意味着 Newton 方向可能不是下降方向，`SimplicialLDLT` 可能失败。

### Tikhonov 正则化

求解器给 reduced Hessian 加上 $\varepsilon I$：

$$
(H_{FF} + \varepsilon I) \Delta x_F = -g_F
$$

这将所有特征值移动 $+\varepsilon$，使矩阵"更正定"。

当 $\varepsilon$ 小（$10^{-8}$）时，对 Newton 方向几乎没有影响。当 $\varepsilon$ 大时，步趋向于 $\Delta x \approx -\varepsilon^{-1} g$（梯度下降）。

### 自适应重试

```cpp
double regularization = std::max(params.regularization, 0.0);
for (int retry = 0; retry < 6; ++retry) {
    const auto reduced_hessian = build_reduced_hessian(..., regularization);
    solved = solve_reduced_system(reduced_hessian, reduced_gradient, reduced_dx);
    if (solved) break;
    regularization = (regularization > 0.0) ? regularization * 10.0 : 1e-8;
}
```

重试序列：$10^{-8} \to 10^{-7} \to 10^{-6} \to 10^{-5} \to 10^{-4} \to 10^{-3}$。

每次重试重新构建矩阵（而非打补丁），因为 `setFromTriplets` 会累加——额外加对角项会导致重复计数。

## 主求解循环

### 入口

```cpp
inline NewtonSolverResult solve(
    IPCState& state,
    const Eigen::VectorXd& x_hat,
    const std::vector<bool>& free_dof_mask,
    const NewtonProblem& problem,
    const NewtonSolverParams& params = {})
```

### 伪代码

```
验证输入
E₀ ← compute_energy(x)
free_dofs ← collect_free_dofs(mask)
if free_dofs 为空: return converged

for k = 0 to max_iterations-1:
    g ← compute_gradient(x)
    zero_fixed_gradient(g, mask)
    if ||g||_inf ≤ ε_g: return converged

    E_k ← compute_energy(x)
    triplets ← compute_hessian_triplets(x)
    g_F ← 提取 reduced gradient

    // 带自适应正则化的求解
    for retry = 0 to 5:
        H_FF ← build_reduced_hessian(triplets, free_dofs, ε)
        solve H_FF · Δx_F = -g_F
        if success: break
        ε ← ε × 10
    if not solved: return failed

    Δx ← expand_reduced_step(Δx_F)
    if ||Δx||_inf ≤ ε_dx: return converged

    // Line search
    d ← g^T · Δx  （方向导数）
    α ← backtracking_line_search(α → E(x + α·Δx), E_k, d)
    if line search failed: return failed

    x ← x + α · Δx

    // 更新后收敛检查
    if ||g(x)||_inf ≤ ε_g or α·||Δx||_inf ≤ ε_dx: return converged

return result（未收敛，达到最大迭代次数）
```

### Line Search 集成

```cpp
const double directional_derivative = gradient.dot(dx);
const auto line_search_result = backtracking_line_search(
    [&state, &problem, &dx](double alpha) {
        const Eigen::VectorXd trial_x = state.x + alpha * dx;
        return problem.compute_energy(trial_x);
    },
    current_energy,
    directional_derivative
);
```

Lambda 捕获：
- `state.x` — 当前位置
- `dx` — 完整 Newton 步（fixed DOF 上为零）
- `problem.compute_energy` — 总能量评估器

对每个 trial $\alpha$，构造 $x_{\text{trial}} = x + \alpha \cdot \Delta x$ 并评估 $E(x_{\text{trial}})$。

### 更新

```cpp
state.x += line_search_result.alpha * dx;
```

`state.x` 原地修改。Newton 循环完成后，`IPCSystem::step()` 用更新后的 `state.x` 计算新速度。

### 日志

每次迭代输出一行：

```
Newton iter=3 energy=1.234567e+01 |g|_inf=2.345678e-03 |dx|_inf=1.234567e-04 alpha=5.000000e-01 success=1
```

## 计算复杂度

| 操作 | 代价 | 说明 |
|------|------|------|
| `compute_gradient` | $O(n + T)$ | $n$ 个顶点，$T$ 个四面体 |
| `compute_hessian_triplets` | $O(T)$ | 每 tet 12x12 block |
| `build_reduced_hessian` | $O(\text{nnz})$ | 过滤 + 重映射 |
| `SimplicialLDLT::compute` | 典型 $O(m^{1.5})$ | 取决于稀疏填充 |
| `SimplicialLDLT::solve` | $O(\text{nnz}_L)$ | 回代 |
| `backtracking_line_search` | $O(k \cdot (n + T))$ | $k$ 次能量评估 |

每次 Newton 迭代的总代价由稀疏分解主导。
每个时间步总代价：$\sim 5\text{-}20$ 次 Newton 迭代 $\times$ 分解代价。

## 错误处理

| 情况 | 响应 |
|------|------|
| 初始能量非有限 | `throw std::runtime_error` |
| 梯度非有限 | `throw std::runtime_error` |
| Newton 步非有限 | `throw std::runtime_error` |
| 更新后 `state.x` 非有限 | `throw std::runtime_error` |
| Hessian 分解失败（所有重试） | `return result`，`converged = false` |
| Line search 失败 | `return result`，`converged = false` |
| 无自由 DOF | `return result`，`converged = true` |
| Hessian triplets 越界 | `throw std::out_of_range` |

非有限值表示上游 bug（能量、梯度或 Hessian 计算），作为硬错误处理。分解和 line search 失败是软错误——返回未收敛结果，由调用方决定如何处理。
