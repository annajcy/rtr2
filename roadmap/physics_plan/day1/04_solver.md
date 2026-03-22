# Phase 3: Newton Solver + Line Search

## 理论基础

### Newton 法作为优化器

**参考：Lec 1 §4.1, 4.3-4.4**

$\nabla E(x) = 0$ 就是 root-finding。Newton 步 = 对能量做二阶近似后取极小值：

$$
\nabla^2 E(x^i) \cdot p = -\nabla E(x^i) \quad \Rightarrow \quad H \Delta x = -g
$$

其中 $\nabla^2 E = M/h^2 + \Delta t^2 \nabla^2 P$。

### Over-Shooting 和 Line Search

**参考：Lec 1 §4.2, 4.4**

Newton 方向可能正确但步长过大（Lec 1 §4.2 的 1D 例子：$E(x)=\sqrt{1+x^2}$，从 $x_0=2$ 跳到 $x_1=-8$）。

解决方案：
1. **SPD Projection**：保证 $H$ 正定 → 搜索方向一定是下降方向
2. **Backtracking Line Search**：$\alpha=1$ 开始，如果能量没降就减半，直到 $E(x+\alpha p) \le E(x) + c \cdot \alpha \cdot g^T p$

每次迭代保证 $E(x^{i+1}) < E(x^i)$ — 单调能量下降。

### Dirichlet 边界条件：DOF Elimination

**参考：Lec 3 全文，重点 §4**

#### 问题：约束优化 vs 无约束优化

隐式 Euler 把时间步推进转化为优化问题：

$$
\min_x \; E(x) = \frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x}) + P(x)
$$

如果没有边界条件，这是一个无约束优化，直接用 Newton 法即可。但物理场景中经常有 **Dirichlet 边界条件**：某些顶点（或顶点的某些分量）的位置被指定为已知值，不参与优化。

数学上，Dirichlet BC 把无约束优化变成了**约束优化**。处理方式有两大类：

| 方法 | 思路 | 优缺点 |
|------|------|--------|
| Penalty（惩罚法） | 加 $\frac{k}{2}\|x_B - \bar{x}_B\|^2$ 到能量里 | 简单但约束不精确，$k$ 大了条件数差 |
| DOF Elimination（消元法） | 直接从优化变量中去掉受约束的 DOF | 精确，矩阵更小，推荐 |

Day 1 采用 **DOF Elimination**。

#### 核心思想：把约束 DOF 从搜索方向中消除

设全局 DOF 向量 $x \in \mathbb{R}^{3n}$，把 DOF 分成两组：

- $x_F$：free DOFs（参与优化）
- $x_C$：constrained DOFs（Dirichlet 指定，不参与优化）

Newton 每一步解 $H \Delta x = -g$。约束的要求是 **$\Delta x_C = 0$**（被约束的 DOF 不允许有增量）。

如果 $\Delta x_C = 0$，那么不管 line search 选什么步长 $\alpha$：

$$
x_C^{new} = x_C + \alpha \cdot 0 = x_C
$$

约束永远不会被破坏。这就是 DOF Elimination 的优美之处：**一次消元，全程安全**。

#### Stick DBC（完全固定）

**物理含义**：顶点 $a$ 的位置完全固定在 $\bar{x}_a$，三个分量都不动。

**约束**：$x_{3a} = \bar{x}_{3a}, \; x_{3a+1} = \bar{x}_{3a+1}, \; x_{3a+2} = \bar{x}_{3a+2}$

即顶点 $a$ 对应的 3 个 DOF 全部被约束：$\Delta x_{3a} = \Delta x_{3a+1} = \Delta x_{3a+2} = 0$。

**消元实现**（行列清零法）：

对线性系统 $H \Delta x = -g$，需要强制 $\Delta x_i = 0$（$i$ 是受约束的 DOF index）：

1. 把 $g_i$ 设为 0（没有驱动力 → 不想让它动）
2. 把 $H$ 的第 $i$ 行和第 $i$ 列的 **非对角元** 清零（解耦：$i$ 不影响其他 DOF，其他 DOF 也不影响 $i$）
3. 把 $H_{ii}$ 设为 1（保证矩阵非奇异）

解出来的 $\Delta x_i$ 自然为 0（因为方程变成了 $1 \cdot \Delta x_i = -0 = 0$）。

**具体例子**：3 个顶点（9 DOF），固定顶点 1（DOF 3,4,5）：

$$
\begin{bmatrix}
H_{00} & H_{01} & 0 & H_{03} & 0 & 0 & H_{06} & H_{07} & H_{08} \\
H_{10} & H_{11} & 0 & H_{13} & 0 & 0 & H_{16} & H_{17} & H_{18} \\
0 & 0 & 1 & 0 & 0 & 0 & 0 & 0 & 0 \\
H_{30} & H_{31} & 0 & H_{33} & 0 & 0 & H_{36} & H_{37} & H_{38} \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 \\
H_{60} & H_{61} & 0 & H_{63} & 0 & 0 & H_{66} & H_{67} & H_{68} \\
H_{70} & H_{71} & 0 & H_{73} & 0 & 0 & H_{76} & H_{77} & H_{78} \\
H_{80} & H_{81} & 0 & H_{83} & 0 & 0 & H_{86} & H_{87} & H_{88}
\end{bmatrix}
\begin{bmatrix} \Delta x_0 \\ \Delta x_1 \\ \Delta x_2 \\ \Delta x_3 \\ \Delta x_4 \\ \Delta x_5 \\ \Delta x_6 \\ \Delta x_7 \\ \Delta x_8 \end{bmatrix}
=
\begin{bmatrix} -g_0 \\ -g_1 \\ 0 \\ -g_3 \\ 0 \\ 0 \\ -g_6 \\ -g_7 \\ -g_8 \end{bmatrix}
$$

注意 DOF 2 是顶点 0 的 z 分量——这里故意没有固定它，只是示意。**真正被固定的是 DOF 3,4,5（顶点 1 的 x,y,z）**，它们对应的行列被清零+对角置 1，梯度置 0。

#### Axis-Aligned Slip DBC（轴对齐滑移）

**物理含义**：顶点 $a$ 在某些轴上固定，其余轴自由。

典型场景：
- **地面滑移**：底部顶点的 $y$ 坐标锁定在 $y=0$，但可以在 $xz$ 平面自由滑动
- **导轨约束**：顶点只能沿某个坐标轴移动（固定另外两个轴）
- **对称面**：沿对称平面法线方向固定，切线方向自由

**约束**：只固定顶点 $a$ 的部分分量。例如 y-slip：

$$
x_{3a+1} = \bar{x}_{3a+1} \quad (\text{y 轴固定})
$$

而 $x_{3a}$（x 轴）和 $x_{3a+2}$（z 轴）是自由的。

**关键洞察：Stick 和 Slip 在 DOF Elimination 框架下是统一的。**

两者的区别仅在于**哪些 DOF 被标记为 constrained**：

| 类型 | 顶点 $a$ 的约束 DOF | free DOF mask |
|------|---------------------|---------------|
| Stick | $\{3a, 3a+1, 3a+2\}$ | `{false, false, false}` |
| Y-Slip | $\{3a+1\}$ | `{true, false, true}` |
| XZ-Slip（导轨沿 y） | $\{3a, 3a+2\}$ | `{false, true, false}` |
| Free | $\emptyset$ | `{true, true, true}` |

消元操作完全相同——对每个 constrained DOF $i$：$g_i = 0$，$H$ 第 $i$ 行列清零，$H_{ii} = 1$。

**Y-Slip 例子**：3 个顶点，顶点 1 做 y-slip（只固定 DOF 4）：

$$
\begin{bmatrix}
H_{00} & H_{01} & H_{02} & H_{03} & 0 & H_{05} & H_{06} & H_{07} & H_{08} \\
H_{10} & H_{11} & H_{12} & H_{13} & 0 & H_{15} & H_{16} & H_{17} & H_{18} \\
H_{20} & H_{21} & H_{22} & H_{23} & 0 & H_{25} & H_{26} & H_{27} & H_{28} \\
H_{30} & H_{31} & H_{32} & H_{33} & 0 & H_{35} & H_{36} & H_{37} & H_{38} \\
0 & 0 & 0 & 0 & 1 & 0 & 0 & 0 & 0 \\
H_{50} & H_{51} & H_{52} & H_{53} & 0 & H_{55} & H_{56} & H_{57} & H_{58} \\
H_{60} & H_{61} & H_{62} & H_{63} & 0 & H_{65} & H_{66} & H_{67} & H_{68} \\
H_{70} & H_{71} & H_{72} & H_{73} & 0 & H_{75} & H_{76} & H_{77} & H_{78} \\
H_{80} & H_{81} & H_{82} & H_{83} & 0 & H_{85} & H_{86} & H_{87} & H_{88}
\end{bmatrix}
\begin{bmatrix} \Delta x_0 \\ \Delta x_1 \\ \Delta x_2 \\ \Delta x_3 \\ \Delta x_4 \\ \Delta x_5 \\ \Delta x_6 \\ \Delta x_7 \\ \Delta x_8 \end{bmatrix}
=
\begin{bmatrix} -g_0 \\ -g_1 \\ -g_2 \\ -g_3 \\ 0 \\ -g_5 \\ -g_6 \\ -g_7 \\ -g_8 \end{bmatrix}
$$

与 stick 对比：只有 DOF 4（顶点 1 的 y）被消元。DOF 3 和 5（顶点 1 的 x 和 z）保持自由，继续参与优化。

解出来后：
- $\Delta x_4 = 0$（y 不动，精确满足约束）
- $\Delta x_3, \Delta x_5 \neq 0$（x, z 可以正常更新）

#### 为什么不需要通用 DBC（任意方向约束）

通用 DBC 允许约束沿任意方向 $n$：$n^T x_a = d$。这需要把约束方向投影到切平面：

$$
\Delta x_a = (I - nn^T) \Delta x_a^{free}
$$

实现上需要修改 Hessian 的投影方式（不能简单清零行列），或者做坐标旋转把约束方向对齐到某个坐标轴。

Day 1 只做 axis-aligned DBC 的原因：
1. **实现简单**：per-DOF 的 bool mask，不需要投影矩阵
2. **覆盖常见场景**：固定天花板、地面滑移、对称面
3. **与 DOF Elimination 天然兼容**：每个 DOF 独立处理，无耦合
4. 通用方向约束留给 Day 3+ 的 contact/friction 模块，那时会有完整的切平面投影机制

#### 初始条件的要求

DOF Elimination 有一个前提：**初始状态必须满足约束**。

如果顶点 $a$ 的 y 坐标应该固定在 0，那 `state.x[3a+1]` 在 `initialize()` 时就必须是 0。否则消元只保证 $\Delta x = 0$（不再变化），但不会把已经错误的位置修正回去。

`IPCSystem::initialize()` 把 `rest_positions` 写入 `state.x`，所以约束顶点的初始位置就是它在 rest shape 中的位置——这自然满足约束。

参考代码：`solid-sim-tutorial/2_dirichlet/time_integrator.py` 中 `search_dir(...)`，`6_inv_free/time_integrator.py` 中的 line search。

---

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

### Axis-Aligned Slip DBC

Day 1 支持 **per-vertex, per-axis** 的 Dirichlet 约束（`TetBody::AxisConstraint`）。

`free_dof_mask` 是 3N 维 bool 向量，由 `TetBody::vertex_constraints` 展开：

```cpp
void build_free_dof_mask() {
    m_free_dof_mask.resize(m_state.dof_count(), true);
    for (const auto& body : m_tet_bodies) {
        for (std::size_t v = 0; v < body.vertex_constraints.size(); ++v) {
            std::size_t global_v = body.info.dof_offset / 3 + v;
            for (int axis = 0; axis < 3; ++axis) {
                if (body.vertex_constraints[v].fixed[axis]) {
                    m_free_dof_mask[3 * global_v + axis] = false;
                }
            }
        }
    }
}
```

三种典型用法：

| 用法 | `AxisConstraint::fixed` | 效果 |
|------|-------------------------|------|
| Stick DBC | `{true, true, true}` | 完全固定，顶点不动 |
| Y-Slip | `{false, true, false}` | y 轴锁定，在 xz 平面自由滑动 |
| Ground Plane | `{false, true, false}` | 顶点可沿地面滑动但不穿透 |

Solver 无需改动——它已经按 per-DOF 处理 `free_dof_mask`。消元逻辑不关心约束的物理含义，只看某个 DOF 是否 free。

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
