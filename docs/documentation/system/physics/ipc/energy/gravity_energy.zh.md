# `gravity_energy.hpp`

`src/rtr/system/physics/ipc/energy/gravity_energy.hpp` 把重力实现成一个独立的线性势能项。

## 物理背景

在优化框架中，外力通过势能进入目标函数。常数外力 $f$ 对应的势能是 $-f^T x$（线性函数）——位移沿力的方向越大，势能越低。

重力就是一个典型的常数外力：每个节点受力 $f_a = m_a \mathbf{g}$。

## 理论推导

### 为什么不把重力折入 $\hat{x}$

有两种等价的处理方式：

**方案 A**：把重力折入惯性预测：$\hat{x} = x_{prev} + hv + h^2 M^{-1} f_g$

**方案 B**：$\hat{x}$ 保持纯惯性预测，重力作为独立能量项

等价性证明：方案 B 的梯度 = $\frac{M}{h^2}(x - \hat{x}) - f_g + \nabla\Psi$。令梯度为零 → $\frac{M}{h^2}(x - (\hat{x} + h^2 M^{-1}f_g)) + \nabla\Psi = 0$，这正好是方案 A。

选择方案 B 的理由：
- $\hat{x}$ 语义清晰（纯运动学）
- 加新外力时只追加能量项，不改 $\hat{x}$ 计算
- 每个能量项可独立 log 调试

### Energy

重力对节点 $a$ 的力为 $f_a = m_a \mathbf{g}$（3D 向量）。对应的势能：

$$
E_{gravity}(x) = -\sum_a m_a \mathbf{g}^T \mathbf{x}_a = -f_g^T x
$$

展开到扁平化的 $3n$ 向量（假设 $\mathbf{g} = (g_x, g_y, g_z)$）：

$$
E_{gravity} = -\sum_{i=0}^{n-1} \big( m_{3i} g_x x_{3i} + m_{3i+1} g_y x_{3i+1} + m_{3i+2} g_z x_{3i+2} \big)
$$

因为 lumped mass 每个顶点的 3 个 DOF 质量相同（$m_{3i} = m_{3i+1} = m_{3i+2}$），所以：

$$
E_{gravity} = -\sum_{i=0}^{n-1} m_{3i} (g_x x_{3i} + g_y x_{3i+1} + g_z x_{3i+2})
$$

这是关于 $x$ 的**线性函数**。

### Gradient

线性函数的导数是常数：

$$
\frac{\partial E_{gravity}}{\partial x_k} = -m_k \cdot g_{k \bmod 3}
$$

向量形式（对每个顶点 $a$）：

$$
\boxed{\nabla_{x_a} E_{gravity} = -m_a \mathbf{g}}
$$

物理含义：重力势能的梯度 = 重力的负值。梯度指向"能量增加的方向"= 向上（和重力相反）。这符合直觉——重力希望物体往下走（降低势能），所以梯度向上。

**关键性质**：梯度是常数（与 $x$ 无关）。可以在 `initialize()` 时预计算一次，每步直接累加。

### Hessian

常数梯度的导数为零：

$$
\boxed{H_{gravity} = 0}
$$

重力不贡献任何 Hessian triplet。这是 `compute_hessian_triplets` 为 no-op 的原因。

物理含义：重力不改变系统的刚度——不管物体在什么位置，重力都一样大。

## 接口

```cpp
struct GravityEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& mass_diag;
        const Eigen::Vector3d& gravity;
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
E = -\sum_i m_i \cdot g_{i \bmod 3} \cdot x_i
$$

```cpp
static double compute_energy(const Input& input) {
    double energy = 0.0;
    for (Eigen::Index i = 0; i < input.x.size(); i += 3) {
        energy -= input.mass_diag[i + 0] * input.gravity.x() * input.x[i + 0];
        energy -= input.mass_diag[i + 1] * input.gravity.y() * input.x[i + 1];
        energy -= input.mass_diag[i + 2] * input.gravity.z() * input.x[i + 2];
    }
    return energy;
}
```

按顶点步进（`i += 3`），每步处理 x/y/z 三个分量。`gravity.x()` 在典型场景下为 0，`gravity.y()` 为 $-9.81$，`gravity.z()` 为 0。

### `compute_gradient`

$$
g_k \mathrel{-}= m_k \cdot g_{k \bmod 3}
$$

```cpp
static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
    for (Eigen::Index i = 0; i < gradient.size(); i += 3) {
        gradient[i + 0] -= input.mass_diag[i + 0] * input.gravity.x();
        gradient[i + 1] -= input.mass_diag[i + 1] * input.gravity.y();
        gradient[i + 2] -= input.mass_diag[i + 2] * input.gravity.z();
    }
}
```

`-=` 因为 $\nabla E = -m \mathbf{g}$（对 y 分量：$-m \cdot (-9.81) = +9.81m$，即梯度向上）。

`+=` 模式：梯度累加到已有值上。假设 $\mathbf{g} = (0, -9.81, 0)$：

- x 分量：$g_{3i+0} -= m \cdot 0 = 0$（不变）
- y 分量：$g_{3i+1} -= m \cdot (-9.81) = g_{3i+1} + 9.81m$（向上的梯度）
- z 分量：$g_{3i+2} -= m \cdot 0 = 0$（不变）

### `compute_hessian_triplets`

```cpp
static void compute_hessian_triplets(const Input& input,
                                     std::vector<Eigen::Triplet<double>>& triplets) {
    validate_inputs(input);
    (void)triplets;  // 刻意 no-op
}
```

Hessian 为零。保留这个函数是为了满足 `Energy` concept 的统一接口，让泛型装配代码不需要为重力单独分支。

## 数值验证

- $\mathbf{g} = (0,0,0)$ 时：$E = 0$，$\nabla E = 0$ ✓
- $\mathbf{g} = (0, -9.81, 0)$ 时：
  - 只有 y 分量有梯度贡献
  - $\nabla E$ 的 y 分量 = $+m \cdot 9.81$（向上，因为往上走势能增加）✓
  - 能量 = $+9.81 \sum_a m_a y_a$（$y$ 越大能量越高）✓
- FD 验证：线性函数的 FD gradient 精确到机器精度（$\sim 10^{-14}$）
- Hessian 不贡献任何 triplet ✓
