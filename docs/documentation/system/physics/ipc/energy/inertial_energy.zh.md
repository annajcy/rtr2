# `inertial_energy.hpp`

`src/rtr/system/physics/ipc/energy/inertial_energy.hpp` 实现了 Backward Euler 优化重构中的纯惯性能项。

## 物理背景

隐式 Euler 时间积分把一步推进转化为优化问题。总能量中的第一项是惯性能——它惩罚偏离惯性预测位置的程度。

直觉：如果没有任何外力或内力，物体应该沿惯性方向匀速运动到 $\hat{x}$。惯性能衡量的就是"实际位置 $x$ 偏离这个惯性预测的程度"，质量越大越难拉偏。

## 理论推导

### 从 Backward Euler 到能量最小化

Backward Euler 更新规则：

$$
M \frac{x^{n+1} - 2x^n + x^{n-1}}{h^2} = f(x^{n+1})
$$

等价于求解（参考 Lec 1 §4.3）：

$$
x^{n+1} = \arg\min_x \; E(x), \qquad E(x) = \underbrace{\frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})}_{E_{inertial}} + h^2 P(x)
$$

其中 $\hat{x} = x^n + h v^n$ 是纯惯性预测位置（不含外力）。

### 惯性能的定义

$$
E_{inertial}(x) = \frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})
$$

$M$ 是 lumped mass 对角矩阵，$h$ 是时间步长。

这是一个关于 $x$ 的**二次型**，展开到每个 DOF：

$$
E_{inertial} = \frac{1}{2h^2} \sum_{i=0}^{3n-1} m_i (x_i - \hat{x}_i)^2
$$

其中 $m_i = \text{mass\_diag}[i]$。

### 梯度

对二次型求导：

$$
\frac{\partial E_{inertial}}{\partial x_i} = \frac{m_i}{h^2}(x_i - \hat{x}_i)
$$

向量形式：

$$
\boxed{\nabla E_{inertial}(x) = \frac{M}{h^2}(x - \hat{x})}
$$

物理含义：梯度方向从 $\hat{x}$ 指向 $x$——如果 $x$ 偏离惯性预测，梯度把它拉回来。在 $x = \hat{x}$ 时梯度为零（惯性预测是惯性能的最小值点）。

### Hessian

梯度再求导：

$$
\frac{\partial^2 E_{inertial}}{\partial x_i \partial x_j} = \begin{cases} m_i / h^2, & i = j \\ 0, & i \neq j \end{cases}
$$

$$
\boxed{H_{inertial} = \frac{M}{h^2}}
$$

这是一个**对角矩阵**，且永远**正定**（质量 > 0，$h > 0$）。

关键性质（Lec 2 §3.2）：惯性 Hessian 不需要 PSD projection，它天然正定。这也是 Newton solver 能工作的基础——即使弹性能的 Hessian 有负特征值，惯性项保证总 Hessian "至少不会太差"。

### $\hat{x}$ 的含义

在本项目中，$\hat{x}$ 是**纯惯性预测**：

$$
\hat{x} = x_{prev} + h \cdot v
$$

不含 $h^2 M^{-1} f_{ext}$ 项。重力由独立的 `GravityEnergy` 处理。

两种方案在数学上等价（把重力折入 $\hat{x}$ vs 作为独立能量项），选择后者是为了语义清晰和可扩展性。

## 接口

```cpp
struct InertialEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& x_hat;
        const Eigen::VectorXd& mass_diag;
        double dt;
    };

    static double compute_energy(const Input& input);
    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient);
    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets);
};
```

## 代码实现

### `compute_energy`

$$
E = \frac{1}{2h^2} \sum_i m_i (x_i - \hat{x}_i)^2
$$

```cpp
static double compute_energy(const Input& input) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    const VectorXd delta = input.x - input.x_hat;
    return 0.5 * inv_dt_sq * input.mass_diag.dot(delta.cwiseProduct(delta));
}
```

`mass_diag.dot(delta.cwiseProduct(delta))` 等价于 $\sum_i m_i \delta_i^2$——先逐元素平方，再与质量做点积，就是加权范数的平方 $\|x - \hat{x}\|_M^2$。

### `compute_gradient`

$$
g_i \mathrel{+}= \frac{m_i}{h^2}(x_i - \hat{x}_i)
$$

```cpp
static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    gradient.array() += inv_dt_sq * input.mass_diag.array() * (input.x - input.x_hat).array();
}
```

使用 Eigen 的 `.array()` 做逐元素运算，避免显式循环。`+=` 模式——梯度累加到已有值上，支持多能量项叠加。

### `compute_hessian_triplets`

$$
H_{ii} = \frac{m_i}{h^2}, \qquad H_{ij} = 0 \; (i \neq j)
$$

```cpp
static void compute_hessian_triplets(const Input& input,
                                     std::vector<Eigen::Triplet<double>>& triplets) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    triplets.reserve(triplets.size() + static_cast<size_t>(input.mass_diag.size()));
    for (Eigen::Index i = 0; i < input.mass_diag.size(); ++i) {
        triplets.emplace_back(i, i, inv_dt_sq * input.mass_diag[i]);
    }
}
```

只输出 $3n$ 个对角 triplet（每个 DOF 一个）。`reserve` 预分配避免 push_back 时频繁 realloc。

## 数值验证

- $x = \hat{x}$ 时：$E = 0$，$\nabla E = 0$ ✓
- $x \neq \hat{x}$ 时：梯度方向指向 $(x - \hat{x})$，即从惯性预测偏离的方向 ✓
- Hessian 永远正定（对角元素 = $m_i / h^2 > 0$）✓
- FD 验证：二次型的 FD gradient 精度应达 $10^{-10}$（中心差分对二次函数是精确的）
