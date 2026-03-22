# Phase 2: 能量模块

## 总能量公式

Day 1 的优化目标：

$$
E(x) = \frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x}) + \Psi_{elastic}(x)
$$

其中：
- $\hat{x} = x_t + h v_t + h^2 M^{-1} f_{ext}$
- $f_{ext}$ 只有重力
- $\Psi_{elastic}$ 是 tet 弹性能

重力通过 $\hat{x}$ 进入系统（作为外力修正预测位置），不需要单独的 gravity energy 项。

---

## File 1: `energy/inertial_energy.hpp`

隐式 Euler 惯性能量。

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
// x_hat = x_prev + dt * v + dt^2 * gravity (component-wise, mass cancels)
for (size_t i = 0; i < state.dof_count(); i += 3) {
    x_hat[i+0] = state.x_prev[i+0] + dt * state.v[i+0];
    x_hat[i+1] = state.x_prev[i+1] + dt * state.v[i+1] + dt * dt * gravity_y;
    x_hat[i+2] = state.x_prev[i+2] + dt * state.v[i+2];
}
```

注意：$h^2 M^{-1} f_{ext}$ 中 $f_{ext} = m g$，所以 $M^{-1} f_{ext} = g$，质量消掉了。

验收：
- 当 `x == x_hat` 时 energy = 0, gradient = 0
- gradient 方向正确（x 偏离 x_hat 时梯度指向回推方向）
- Hessian 对角值正确

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
