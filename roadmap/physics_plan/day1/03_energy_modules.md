# Phase 2: 能量模块

## 理论基础

### 优化重构：隐式 Euler = 能量最小化

**参考：Lec 1 §4.3，Lec 2 §3.1**

Backward Euler 等价于最小化增量势能：

$$
x^{n+1} = \arg\min_x \; \frac{1}{2}\|x - \tilde{x}^n\|_M^2 + \Delta t^2 \, P(x)
$$

物理直觉：
- **惯性项**：惩罚偏离惯性预测位置的程度——质量越大越难拉偏
- **势能项** $\Delta t^2 P$：弹性能 + 重力势能——希望位置处于低能构形
- $\Delta t^2$ 的来源（Lec 2 §3.1）：力→速度增量（$\times h$）→位移增量（$\times h$），两次 = $h^2$

注意：Lec 2 §3.2 指出惯性 Hessian 是质量矩阵（永远正定），不需要 PSD projection。

### Deformation Gradient $F$

**参考：Lec 7 §1.2-1.4**

$F = \partial x / \partial X$ 描述"点附近的一小块材料怎样被拉、压、剪、转"。离散 FEM 中：

$$
D_s = [x_1 - x_0, \; x_2 - x_0, \; x_3 - x_0], \quad F = D_s \cdot D_m^{-1}
$$

直觉验证：没变形 → $F=I$；纯旋转 → $F=R$；$J = \det(F)$ = 体积变化比。

### Strain Energy 和材料模型

**参考：Lec 7 §4-5**

好的 $\Psi(F)$ 应满足：rigid null space（$\Psi(R)=0$）、rotation invariance（$\Psi(RF)=\Psi(F)$）、isotropy（$\Psi(FR)=\Psi(F)$）。

Day 1 选用 **Fixed Corotated**（Lec 7 §5.2 变体）：$\Psi_{FC}(F) = \mu\|F-R\|_F^2 + \frac{\lambda}{2}(J-1)^2$。选它而非 Neo-Hookean：实现更简单（不需要 $\ln J$），数值更稳（$J \le 0$ 不会炸）。

Lame 参数（Lec 7 §5.5）：$\mu = E/[2(1+\nu)]$，$\lambda = E\nu/[(1+\nu)(1-2\nu)]$。

### PK1 Stress 和从 $F$ 到 $x$ 的链式法则

**参考：Lec 8 §2, 5，Lec 10 §5.4-5.6**

First Piola-Kirchhoff stress：$P = \partial\Psi/\partial F$。从 $P$ 到节点力（Lec 10 §5.4）：

$$
f^{int}_{e,a} = -V_e \cdot P_e \cdot \nabla N_a
$$

其中 $\nabla N_a$ 由 $D_m^{-1}$ 决定。对 3D 四面体，$\nabla N_1 = D_m^{-T}[-1,-1,-1]^T$，$\nabla N_2 = D_m^{-T}[1,0,0]^T$，等。且 $\sum_a \nabla N_a = 0$（partition of unity 推论）。

### Hessian 和 PSD Projection

**参考：Lec 8 §3-4**

Stress derivative $\partial P/\partial F$ 在 diagonal space 分解为 A block 和 B blocks（Lec 8 §4.3）。Hessian 从 $F$ 映射到 $x$：$\partial^2\Psi/\partial x^2 = (\partial F/\partial x)^T (\partial^2\Psi/\partial F^2) (\partial F/\partial x)$。线性四面体中 $F$ 对 $x$ 是线性的，没有额外二阶项。

PSD Projection（Lec 8 §4.6）：对负特征值 clamp 到零。实际在单元级别（12×12）做更高效。Day 1 简化版先用全局对角 regularization $H + \epsilon I$。

参考代码：`solid-sim-tutorial/6_inv_free/`。

---

## 总能量公式

Day 1 的优化目标：

$$
E(x) = \underbrace{\frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})}_{E_{inertial}} + \underbrace{E_{gravity}(x)}_{-f_g^T x} + \underbrace{\Psi_{elastic}(x)}_{弹性能}
$$

其中：
- $\hat{x} = x_t + h v_t$ —— **纯惯性预测位置**，不含外力修正
- $E_{gravity}(x) = -\sum_i m_i \mathbf{g}^T \mathbf{x}_i$ —— 重力势能，作为独立能量项
- $\Psi_{elastic}$ 是 tet 弹性能

### 为什么把重力作为独立能量项

两种处理方式在数学上等价：

**方案 A（折入 $\hat{x}$）：** $\hat{x} = x_t + hv_t + h^2 M^{-1}f_{ext}$，$E = \frac{1}{2h^2}(x-\hat{x})^T M(x-\hat{x}) + \Psi_{elastic}$

**方案 B（独立 gravity energy）：** $\hat{x} = x_t + hv_t$，$E = \frac{1}{2h^2}(x-\hat{x})^T M(x-\hat{x}) + E_{gravity}(x) + \Psi_{elastic}$

等价性证明：展开方案 B 的梯度，$\nabla E_{inertial} = \frac{M}{h^2}(x-\hat{x})$，$\nabla E_{gravity} = -f_g$。令梯度为零：$\frac{M}{h^2}(x-\hat{x}) - f_g + \nabla\Psi = 0$，即 $\frac{M}{h^2}(x - (\hat{x} + h^2 M^{-1}f_g)) + \nabla\Psi = 0$，与方案 A 完全相同。

选择方案 B 的理由：
- **语义清晰**：$\hat{x}$ 就是"如果没有任何力作用、只凭惯性会到哪"，纯运动学概念
- **可扩展**：Day 3+ 加 barrier、friction 等外力时，统一以独立能量项追加，不需要反复修改 $\hat{x}$ 计算
- **可调试**：每个能量项的贡献可以独立 log，方便定位数值问题

---

## File 1: `energy/inertial_energy.hpp`

纯惯性能量，不含外力。

### 接口

```cpp
namespace rtr::system::physics::ipc {

struct InertialEnergy {
    // E = 0.5/h^2 * (x - x_hat)^T M (x - x_hat)
    static double compute_energy(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& x_hat,
        const Eigen::VectorXd& mass_diag,
        double dt
    );

    // g_i = M_i / h^2 * (x_i - x_hat_i)
    static void compute_gradient(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& x_hat,
        const Eigen::VectorXd& mass_diag,
        double dt,
        Eigen::VectorXd& gradient  // += 模式
    );

    // H = M / h^2  (对角)
    static void compute_hessian_triplets(
        const Eigen::VectorXd& mass_diag,
        double dt,
        std::vector<Eigen::Triplet<double>>& triplets  // += 模式
    );
};

}
```

### 实现要点

- energy: `0.5 / (dt*dt) * sum_i( mass_diag[i] * (x[i] - x_hat[i])^2 )`
- gradient: `mass_diag[i] / (dt*dt) * (x[i] - x_hat[i])` for each DOF
- Hessian: 对角矩阵，`H(i,i) = mass_diag[i] / (dt*dt)`

### $\hat{x}$ 计算

在 `IPCSystem` 中计算，不在 InertialEnergy 里：

```cpp
// x_hat = x_prev + dt * v  （纯惯性预测，不含重力）
m_x_hat = m_state.x_prev + m_config.dt * m_state.v;
```

注意：这里 $\hat{x}$ **不含** $h^2 M^{-1} f_{ext}$ 项。重力由独立的 GravityEnergy 处理。

验收：
- 当 `x == x_hat` 时 energy = 0, gradient = 0
- gradient 方向正确（x 偏离 x_hat 时梯度指向回推方向）
- Hessian 对角值正确

---

## File 2: `energy/gravity_energy.hpp`

重力势能，作为独立的能量项参与优化。

### 理论

重力对节点 $a$ 的力为 $f_a = m_a \mathbf{g}$，其中 $\mathbf{g} = (0, -9.81, 0)^T$。

在优化框架中，常数力对应线性势能：

$$
E_{gravity}(x) = -f_g^T x = -\sum_a m_a \mathbf{g}^T \mathbf{x}_a
$$

展开（假设 $\mathbf{g} = (0, g_y, 0)$）：

$$
E_{gravity} = -\sum_a m_a \, g_y \, x_{a,y}
$$

这是关于 $x$ 的线性函数，所以：
- 梯度是常数（与 $x$ 无关）
- Hessian 为零

### 接口

```cpp
namespace rtr::system::physics::ipc {

struct GravityEnergy {
    // E = -sum_i( mass_diag[i] * g[i%3] * x[i] )
    static double compute_energy(
        const Eigen::VectorXd& x,
        const Eigen::VectorXd& mass_diag,
        const Eigen::Vector3d& gravity
    );

    // g_i = -mass_diag[i] * gravity[i%3]  (常数，与 x 无关)
    static void compute_gradient(
        const Eigen::VectorXd& mass_diag,
        const Eigen::Vector3d& gravity,
        Eigen::VectorXd& gradient  // += 模式
    );

    // Hessian = 0  (线性能量没有二阶导)
    // 不需要 compute_hessian_triplets —— 不贡献任何 triplet
};

}
```

### 实现要点

```cpp
static double compute_energy(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& mass_diag,
    const Eigen::Vector3d& gravity
) {
    double E = 0.0;
    for (Eigen::Index i = 0; i < x.size(); i += 3) {
        E -= mass_diag[i+0] * gravity.x() * x[i+0];
        E -= mass_diag[i+1] * gravity.y() * x[i+1];
        E -= mass_diag[i+2] * gravity.z() * x[i+2];
    }
    return E;
}

static void compute_gradient(
    const Eigen::VectorXd& mass_diag,
    const Eigen::Vector3d& gravity,
    Eigen::VectorXd& gradient
) {
    for (Eigen::Index i = 0; i < gradient.size(); i += 3) {
        gradient[i+0] -= mass_diag[i+0] * gravity.x();
        gradient[i+1] -= mass_diag[i+1] * gravity.y();
        gradient[i+2] -= mass_diag[i+2] * gravity.z();
    }
}
```

### 性能说明

gravity gradient 是常数（不依赖 $x$），可以在 `initialize()` 时预计算一次，存为 `m_gravity_gradient`，每步直接 `gradient += m_gravity_gradient`。这是一个可选优化，Day 1 不必做。

验收：
- gravity = (0,0,0) 时 energy = 0, gradient = 0
- gravity = (0,-9.81,0) 时，gradient 在 y 分量为 $+m_a \times 9.81$（能量对 y 的导数，指向上方，因为往下走能量减小）
- Hessian 不贡献任何 triplet

---

## File 2: `energy/tet_material_model.hpp`

材料模型抽象。

```cpp
namespace rtr::system::physics::ipc {

struct TetMaterialModel {
    virtual ~TetMaterialModel() = default;

    // 单个 tet 的弹性能
    virtual double compute_energy(
        const Eigen::Matrix3d& F,       // deformation gradient
        double rest_volume,
        double youngs_modulus,
        double poisson_ratio
    ) const = 0;

    // 单个 tet 对 F 的梯度 (9-vector, 对 vec(F) 的导数)
    virtual Eigen::Matrix3d compute_pk1(
        const Eigen::Matrix3d& F,
        double rest_volume,
        double youngs_modulus,
        double poisson_ratio
    ) const = 0;

    // 单个 tet 对 F 的 Hessian (9x9)
    virtual Eigen::Matrix<double, 9, 9> compute_hessian(
        const Eigen::Matrix3d& F,
        double rest_volume,
        double youngs_modulus,
        double poisson_ratio
    ) const = 0;
};

}
```

### F 的计算

给定 tet `[v0, v1, v2, v3]`，当前位置 `x`：

```
Ds.col(0) = x[v1] - x[v0]
Ds.col(1) = x[v2] - x[v0]
Ds.col(2) = x[v3] - x[v0]
F = Ds * Dm_inv
```

这个计算放在装配层，不放在材料模型里。

### 从 F 的导数到 DOF 的导数

材料模型返回对 `F` 的 PK1 stress (3x3) 和 Hessian (9x9)。

装配层负责通过链式法则映射到全局 DOF：

```
dE/dx_i = V * P * (dF/dx_i)     // P = PK1 stress
d^2E/dx_i dx_j = V * (dF/dx_i)^T * d^2Psi/dF^2 * (dF/dx_j)
```

其中 `dF/dx` 由 `Dm_inv` 决定，是常数矩阵。

---

## File 3: `energy/tet_fixed_corotated_energy.hpp`

Day 1 唯一 concrete material。

### Fixed Corotated 模型

给定 F，做极分解 `F = R * S`：
- `R` 是旋转部分
- `S` 是对称正定部分

能量密度：

$$
\Psi(F) = \mu \|F - R\|_F^2 + \frac{\lambda}{2} (J - 1)^2
$$

其中：
- $\mu = E / (2(1+\nu))$
- $\lambda = E\nu / ((1+\nu)(1-2\nu))$
- $J = \det(F)$

### 实现要点

- 极分解用 SVD：`F = U * Sigma * V^T`，`R = U * V^T`
- PK1 stress: $P = 2\mu(F - R) + \lambda(J-1)J F^{-T}$
- Hessian: 需要 SVD 导数，Day 1 可以先用数值近似或简化版本
  - 如果 Hessian 推导太耗时，Day 1 可以先用 **有限差分近似 Hessian**（每个 tet 9x9 矩阵，finite diff PK1）
  - 但要在代码里标注 `// TODO: analytic Hessian`

### Day 1 最低要求

- energy: 必须正确
- PK1 (gradient): 必须正确
- Hessian: 可以用 FD 近似，但必须存在

验收：
- 单 tet 在 identity F 时 energy = 0
- 小扰动下 PK1 与 FD gradient 一致
- 压缩/拉伸方向的能量变化符合物理直觉

---

## File 4: `energy/tet_elastic_assembler.hpp`

从单元级能量/梯度/Hessian 装配到全局系统。

```cpp
namespace rtr::system::physics::ipc {

struct TetElasticAssembler {
    static double compute_energy(
        const TetBody& body,
        const IPCState& state,
        const TetMaterialModel& material
    );

    static void compute_gradient(
        const TetBody& body,
        const IPCState& state,
        const TetMaterialModel& material,
        Eigen::VectorXd& gradient  // += 模式
    );

    static void compute_hessian_triplets(
        const TetBody& body,
        const IPCState& state,
        const TetMaterialModel& material,
        std::vector<Eigen::Triplet<double>>& triplets  // += 模式
    );
};

}
```

### 装配逻辑

对每个 tet `t`：

1. 取 4 个顶点的当前位置
2. 计算 `Ds`, `F = Ds * body.Dm_inv[t]`
3. 调用 `material.compute_energy(F, body.rest_volumes[t], ...)`
4. 调用 `material.compute_pk1(...)` 得到 PK1
5. 通过 `dF/dx` 映射到 4 个顶点的 12 个 DOF
6. 写入全局 gradient
7. 同理 Hessian -> 12x12 局部矩阵 -> scatter to global triplets

### 关键：dF/dx 映射

对于 tet `[v0, v1, v2, v3]`：

```
F = Ds * Dm_inv

其中 Ds = [x1-x0, x2-x0, x3-x0]

所以 dF/dx1 的贡献 = e1^T * Dm_inv (对 x1 的 x 分量)
     dF/dx0 = -(dF/dx1 + dF/dx2 + dF/dx3)
```

具体地，gradient 对 v0 的贡献：

```
grad_v0 = -V * P * Dm_inv^T * [1;1;1]  // 简化写法，实际按列累加
```

准确公式见 IPC 论文附录或 FEM 教材。

验收：
- 全局 gradient 大小 = `state.dof_count()`
- 装配后 gradient 不含 NaN
- 单 tet energy 和调 material.compute_energy 直接算的结果一致
