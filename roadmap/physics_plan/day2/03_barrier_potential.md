# Phase 3: Barrier Potential

## 目标

实现 IPC 的 barrier energy，使接触力自然地从优化框架中涌现，而不是显式的碰撞检测 + 碰撞响应。

## IPC barrier 的核心思想

传统碰撞处理：检测穿透 → 施加罚力或约束力。问题在于检测和响应是两个独立步骤，容易出现"刚好穿过检测窗口"的情况。

IPC 的做法：把接触势能加入总优化目标。当两个物体靠近时，barrier energy 快速增大，阻止优化器选择穿透方向。

$$E_{total}(x) = E_{inertial}(x) + E_{gravity}(x) + \Psi_{elastic}(x) + \kappa \sum_{(i,j) \in \mathcal{C}} b(d_{ij}(x), \hat{d})$$

其中 $\kappa$ 是 barrier stiffness，$\hat{d}$ 是激活距离阈值，$b$ 是标量 barrier 函数。

---

## 标量 Barrier 函数

### 定义

IPC 论文使用的 barrier 函数：

$$b(d, \hat{d}) = \begin{cases} -(d - \hat{d})^2 \ln(d / \hat{d}) & \text{if } 0 < d < \hat{d} \\ 0 & \text{if } d \geq \hat{d} \end{cases}$$

其中 $d$ 是距离平方（$d = d^2_{physical}$，或者用物理距离 $d_{physical}$，取决于实现选择）。

**重要选择**：IPC 原始论文中 $d$ 是距离（不是距离平方）。但很多实现用距离平方作为 barrier 输入以避免开方。Day 2 推荐**使用距离平方**，这样和 Phase 1 的距离函数直接对接。

如果使用距离平方，令 $s = d^2$：

$$b(s, \hat{s}) = \begin{cases} -(s - \hat{s})^2 \ln(s / \hat{s}) & \text{if } 0 < s < \hat{s} \\ 0 & \text{if } s \geq \hat{s} \end{cases}$$

### 一阶导数

$$\frac{\partial b}{\partial s} = -(s - \hat{s})\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right)$$

### 二阶导数

$$\frac{\partial^2 b}{\partial s^2} = -\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right) - (s - \hat{s})\left(\frac{2}{s} + \frac{\hat{s}}{s^2}\right)$$

化简：

$$\frac{\partial^2 b}{\partial s^2} = -2\ln(s/\hat{s}) - \frac{(s - \hat{s})(3s - \hat{s})}{s^2}$$

### 数值注意

- 当 $s \to 0$ 时 $b \to +\infty$，$b' \to -\infty$（阻止穿透）
- 当 $s = \hat{s}$ 时 $b = 0$，$b' = 0$（$C^1$ 连续过渡到零）
- 当 $s > \hat{s}$ 时 $b = 0$（不激活）

实现中需要防止 $s \leq 0$ 进入 barrier（这是物理上不应发生的，但数值上可能出现浮点误差）：

```cpp
if (s <= 0.0) {
    // 已经穿透，返回极大值或 throw
}
```

---

## 从标量 barrier 到全局 DOF

### 链式法则

barrier 对全局 DOF $x$ 的贡献通过链式法则：

$$\frac{\partial B}{\partial x} = \kappa \sum_{(i,j)} \frac{\partial b}{\partial s} \cdot \frac{\partial s}{\partial x}$$

$$\frac{\partial^2 B}{\partial x^2} = \kappa \sum_{(i,j)} \left(\frac{\partial^2 b}{\partial s^2} \cdot \frac{\partial s}{\partial x} \cdot \frac{\partial s}{\partial x}^T + \frac{\partial b}{\partial s} \cdot \frac{\partial^2 s}{\partial x^2}\right)$$

其中 $s = d^2(x)$ 是距离平方，$\frac{\partial s}{\partial x}$ 和 $\frac{\partial^2 s}{\partial x^2}$ 来自 Phase 1 的几何距离模块。

### 局部到全局装配

每个接触对 $(i,j)$ 涉及 12 个局部 DOF（PT 或 EE）。局部 12×12 Hessian 需要散装配到全局 $3N \times 3N$ 稀疏矩阵。

装配路径：
1. 计算局部距离平方 $s$ 和局部 gradient/Hessian
2. 计算标量 barrier $b(s)$, $b'(s)$, $b''(s)$
3. 局部 barrier gradient = $\kappa \cdot b'(s) \cdot \nabla_x s$（12 维）
4. 局部 barrier Hessian = $\kappa \cdot (b''(s) \cdot \nabla_x s \cdot \nabla_x s^T + b'(s) \cdot \nabla^2_x s)$（12×12）
5. 对局部 Hessian 做 PSD projection
6. 通过 vertex_map 散装到全局 gradient 和 triplet 列表

### PSD Projection

接触 Hessian 可能不正定（距离函数的 Hessian 不一定正定）。必须对每个局部 12×12 Hessian 做 PSD 投影：

```cpp
// 特征分解
Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> solver(local_hessian);
auto eigenvalues = solver.eigenvalues();
auto eigenvectors = solver.eigenvectors();

// 截断负特征值
for (int i = 0; i < 12; ++i) {
    eigenvalues[i] = std::max(eigenvalues[i], 0.0);
}

// 重组
local_hessian = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
```

这保证每个接触对的 Hessian 贡献是半正定的，Newton 方向是下降方向。

---

## 参数选择

### $\hat{d}$（激活距离阈值）

| 场景 | 推荐 $\hat{d}^2$ | 说明 |
|------|------------------|------|
| Day 2 demo | `0.01` ($\hat{d} \approx 0.1$) | 适合 spacing=0.2~0.3 的 tet block |
| 一般场景 | `(0.01 \cdot bbox\_diag)^2` | 相对于场景尺度 |

### $\kappa$（barrier stiffness）

Day 2 先用固定 $\kappa$：

| 场景 | 推荐 $\kappa$ | 说明 |
|------|--------------|------|
| Day 2 demo | `1e3` ~ `1e5` | 从小开始调，太大会让 Newton 收敛变慢 |

后续可以做 adaptive $\kappa$（IPC 论文中的 Algorithm 1），但 Day 2 不做。

---

## 实现结构

```cpp
// 标量 barrier 函数（纯数学）
namespace barrier {
    double energy(double s, double s_hat);
    double gradient(double s, double s_hat);  // db/ds
    double hessian(double s, double s_hat);   // d²b/ds²
}

// barrier potential 装配到全局
struct BarrierPotentialConfig {
    double dhat_squared{0.01};   // s_hat
    double kappa{1e4};
};

double compute_barrier_energy(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config
);

void compute_barrier_gradient(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    Eigen::VectorXd& gradient  // accumulate
);

void compute_barrier_hessian_triplets(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    std::vector<Eigen::Triplet<double>>& triplets  // accumulate
);
```

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/contact/barrier_function.hpp` | 标量 $b(s, \hat{s})$ 及一二阶导数 |
| `src/rtr/system/physics/ipc/contact/barrier_potential.hpp` | 全局 barrier energy/gradient/Hessian 装配 |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/contact/barrier_function_test.cpp` | 标量 barrier fd check、边界行为 |
| `test/system/physics/ipc/contact/barrier_potential_test.cpp` | 单个 PT/EE 对的 barrier gradient fd check |
