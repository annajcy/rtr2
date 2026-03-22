# Phase 3: Newton Solver + Line Search

## File 1: `solver/line_search.hpp`

Day 1 只做基础回溯 Armijo line search，不做 CCD。

### 接口

```cpp
namespace rtr::system::physics::ipc {

struct LineSearchResult {
    double alpha{0.0};
    double energy{0.0};
    bool success{false};
};

// energy_fn: 给定 alpha，返回 E(x + alpha * dx)
using EnergyFunction = std::function<double(double alpha)>;

LineSearchResult backtracking_line_search(
    EnergyFunction energy_fn,
    double current_energy,
    double directional_derivative,  // g^T * dx，应为负值
    double alpha_init = 1.0,        // 初始步长
    double shrink = 0.5,            // 缩放因子
    double armijo_c = 1e-4,         // Armijo 常数
    int max_iterations = 20         // 最大回溯次数
);

}
```

### 实现

```
alpha = alpha_init
for i in 0..max_iterations:
    trial_energy = energy_fn(alpha)
    if trial_energy <= current_energy + armijo_c * alpha * directional_derivative:
        return {alpha, trial_energy, true}
    alpha *= shrink
return {alpha, trial_energy, false}
```

### Day 4 扩展点

Day 4 加 CCD 时，在调 `backtracking_line_search` 之前先用 CCD 计算 `alpha_max`，然后 `alpha_init = min(1.0, alpha_max)`。

验收：
- full step 降能时返回 alpha = 1.0
- full step 不降能时自动缩步
- directional_derivative >= 0 时立即返回 failure

---

## File 2: `solver/newton_solver.hpp`

Day 1 核心：最小 Newton 求解器。

### 参数

```cpp
namespace rtr::system::physics::ipc {

struct NewtonSolverParams {
    int max_iterations{50};
    double gradient_tolerance{1e-6};    // ||g||_inf
    double dx_tolerance{1e-8};          // ||dx||_inf
    double regularization{1e-8};        // 对角加 epsilon
    bool use_psd_projection{false};     // Day 1 先关闭，Day 2 开
};

struct NewtonSolverResult {
    int iterations{0};
    double final_gradient_norm{0.0};
    double final_energy{0.0};
    bool converged{false};
};

}
```

### 主流程

```cpp
NewtonSolverResult solve(
    IPCState& state,
    const Eigen::VectorXd& x_hat,
    const std::vector<bool>& free_dof_mask,  // Dirichlet 消元
    /* energy/gradient/hessian assembler callback */
    const NewtonSolverParams& params
);
```

每次迭代：

1. 装配总 energy `E(x)`
2. 装配总 gradient `g`
3. 装配总 Hessian triplets -> `Eigen::SparseMatrix<double> H`
4. 对角 regularization: `H += epsilon * I`
5. **Dirichlet 消元**：
   - 把 fixed DOF 对应的 gradient 行清零
   - 把 fixed DOF 对应的 Hessian 行和列清零，对角置 1
   - 或者用 reduced system 只解 free DOFs（推荐，更干净）
6. 解 `H dx = -g`（用 `Eigen::SimplicialLDLT`）
7. 检查 `g^T * dx < 0`（下降方向）
8. 调 `backtracking_line_search`
9. 更新 `x += alpha * dx`
10. 检查收敛

### Dirichlet 消元策略

推荐 Day 1 用 **行列清零 + 对角置 1** 方式，因为更简单：

```cpp
for (int i : fixed_dofs) {
    gradient[i] = 0.0;
    // Hessian: 在 triplet 阶段就不写 fixed DOF 的 off-diagonal，对角写 1.0
}
```

这样解出来的 `dx` 在 fixed DOF 处自然为 0。

### 线性求解

```cpp
Eigen::SparseMatrix<double> H(n, n);
H.setFromTriplets(triplets.begin(), triplets.end());
Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
solver.compute(H);
if (solver.info() != Eigen::Success) {
    // 加更强 regularization 重试一次
}
Eigen::VectorXd dx = solver.solve(-gradient);
```

### 日志输出

每次迭代至少 log：

- iteration number
- energy
- `||g||_inf`
- `||dx||_inf`
- line search alpha
- line search success

用 `spdlog` 的 `utils::get_logger("physics.ipc.solver")`。

### Newton 不收敛时的 fallback

- 达到 `max_iterations` 时停止，返回 `converged = false`
- 不回退到上一步，而是保留当前最好的 x
- log warning
- 调用方（IPCSystem）根据 converged flag 决定是否继续步进

验收：
- 二次能量（纯 inertial）1 步收敛
- tet elastic 能量数步内梯度范数下降
- fixed DOF 在求解后保持不变
- 不出 NaN
