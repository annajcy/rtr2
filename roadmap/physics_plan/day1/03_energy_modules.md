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

### PK1 Stress 和从 $F$ 到节点力 $f^{int}_{e,a}$

**参考：Lec 8 §2, 5，Lec 10 §5.4-5.6**

First Piola-Kirchhoff stress：$P = \partial\Psi/\partial F$。最终目标是从 $P$ 推出**每个节点受到的弹性力**。完整推导链路如下：

#### Step 1: 单元弹性能 → 节点力

一个四面体 $e$ 的弹性能为（线性 tet 中 $F$ 是常量，积分退化为乘体积）：

$$
\Psi_e = V_e \cdot \psi(F_e)
$$

节点 $a$ 受到的弹性力 = 能量对该节点坐标的负梯度：

$$
f^{int}_{e,a} = -\frac{\partial \Psi_e}{\partial x_a} = -V_e \frac{\partial \psi(F_e)}{\partial x_a}
$$

#### Step 2: 链式法则 — 通过 $F$ 转接

$\psi$ 的自变量是 $F$，不是直接的 $x_a$。用链式法则：

$$
\frac{\partial \psi}{\partial x_a} = \frac{\partial \psi}{\partial F} : \frac{\partial F}{\partial x_a} = P : \frac{\partial F}{\partial x_a}
$$

其中 $:$ 是 Frobenius 内积（$A:B = \sum_{ij} A_{ij} B_{ij}$），$P = \partial\psi/\partial F$ 就是 PK1 stress。

#### Step 3: 计算 $\partial F / \partial x_a$

回顾 $F = D_s \cdot D_m^{-1}$，其中 $D_s = [x_1-x_0, \; x_2-x_0, \; x_3-x_0]$。

**关键观察**：$D_s$ 的第 $k$ 列就是 $x_{k+1} - x_0$。所以移动 $x_a$（$a \in \{1,2,3\}$）只改变 $D_s$ 的第 $(a{-}1)$ 列，其余列不变。

给 $x_a$ 一个微小扰动 $\delta x_a$（3D 向量），$D_s$ 的变化是：

$$
\delta D_s = \text{第 } (a{-}1) \text{ 列变成 } \delta x_a，\text{其余列为零} = \delta x_a \cdot e_{a-1}^T
$$

其中 $e_{a-1}$ 是标准基向量（$e_0 = [1,0,0]^T$ 等）。这里 $\delta x_a \cdot e_{a-1}^T$ 是外积，给出一个 3×3 矩阵。

代入 $F = D_s \cdot D_m^{-1}$：

$$
\delta F = \delta D_s \cdot D_m^{-1} = (\delta x_a \cdot e_{a-1}^T) \cdot D_m^{-1} = \delta x_a \cdot \underbrace{(e_{a-1}^T D_m^{-1})}_{\text{1×3 行向量}}
$$

$e_{a-1}^T D_m^{-1}$ 就是 $D_m^{-1}$ 的第 $(a{-}1)$ 行，转置后等于 $D_m^{-T}$ 的第 $(a{-}1)$ 列。定义 $\nabla N_a \equiv D_m^{-T} e_{a-1}$，则：

$$
\delta F = \delta x_a \otimes \nabla N_a \qquad \text{（外积：} (\delta F)_{ij} = (\delta x_a)_i (\nabla N_a)_j \text{）}
$$

**用数值例子验证**：

取参考四面体 $X_0=(0,0,0)$，$X_1=(1,0,0)$，$X_2=(0,1,0)$，$X_3=(0,0,1)$。

此时 $D_m = I$，$D_m^{-1} = I$，$D_m^{-T} = I$。

当前位置 $x_0=(0,0,0)$，$x_1=(2,0,0)$，$x_2=(0,1,0)$，$x_3=(0,0,1)$（$x_1$ 方向拉伸了 2 倍）。

$$
D_s = \begin{bmatrix}2&0&0\\0&1&0\\0&0&1\end{bmatrix}, \quad F = D_s \cdot I = \begin{bmatrix}2&0&0\\0&1&0\\0&0&1\end{bmatrix}
$$

现在扰动 $x_1$：在 y 方向推一下，$\delta x_1 = (0, \epsilon, 0)$。

**直接算**：新的 $D_s' = [(2, \epsilon, 0), \; (0,1,0), \; (0,0,1)]$，所以

$$
\delta D_s = \begin{bmatrix}0&0&0\\\epsilon&0&0\\0&0&0\end{bmatrix}, \quad \delta F = \delta D_s \cdot I = \begin{bmatrix}0&0&0\\\epsilon&0&0\\0&0&0\end{bmatrix}
$$

只有 $\delta F_{21} = \epsilon$，其余全零。

**用公式验证**：$a=1$，$\nabla N_1 = D_m^{-T} e_0 = I \cdot [1,0,0]^T = [1,0,0]^T$。

$$
\delta F = \delta x_1 \otimes \nabla N_1 = \begin{bmatrix}0\\\epsilon\\0\end{bmatrix} \otimes \begin{bmatrix}1\\0\\0\end{bmatrix} = \begin{bmatrix}0&0&0\\\epsilon&0&0\\0&0&0\end{bmatrix} \quad \checkmark
$$

**物理含义**：把 $x_1$（原来在 x 轴方向的顶点）往 y 方向推，导致"参考基向量 $e_0=[1,0,0]$ 的像"多了一个 y 分量——这就是 $F_{21}$ 变化的几何意义。

#### Step 3.5: 从 $\delta F$ 到能量变化

把 $\delta F = \delta x_a \otimes \nabla N_a$ 代入链式法则 $\delta\psi = P : \delta F$：

$$
\delta\psi = P : (\delta x_a \otimes \nabla N_a) = \sum_{ij} P_{ij} (\delta x_a)_i (\nabla N_a)_j = (P \cdot \nabla N_a)^T \cdot \delta x_a
$$

最后一步利用了矩阵-向量乘法的定义：$(P \cdot \nabla N_a)_i = \sum_j P_{ij} (\nabla N_a)_j$。

所以 $\partial\psi / \partial x_a = P \cdot \nabla N_a$（一个 3D 向量），代入 Step 1：

$$
\boxed{f^{int}_{e,a} = -V_e \cdot P_e \cdot \nabla N_a}
$$

**继续用上面的数值例子**：假设某材料在 $F = \text{diag}(2,1,1)$ 下的 PK1 stress 为 $P = \text{diag}(p_1, 0, 0)$（x 方向有应力，yz 无）。

$$
f^{int}_{e,1} = -V_e \cdot P \cdot \nabla N_1 = -V_e \begin{bmatrix}p_1&0&0\\0&0&0\\0&0&0\end{bmatrix} \begin{bmatrix}1\\0\\0\end{bmatrix} = -V_e \begin{bmatrix}p_1\\0\\0\end{bmatrix}
$$

顶点 1 沿 x 轴受到回弹力，方向与拉伸相反。符合物理直觉。

#### Step 4: $\nabla N_a$ 的具体值

$\nabla N_a$ 是 shape function $N_a(X)$ 对参考坐标 $X$ 的梯度。对线性四面体，$N_a$ 是仿射函数（重心坐标），所以 $\nabla N_a$ 是常向量。

从 $F = D_s \cdot D_m^{-1}$ 和 $F = \sum_a x_a \otimes \nabla N_a$ 对比，可以读出：

记 $G = D_m^{-T}$（$D_m$ 逆的转置），$G$ 的三列分别记为 $g_1, g_2, g_3$，则：

$$
\nabla N_1 = g_1, \quad \nabla N_2 = g_2, \quad \nabla N_3 = g_3
$$

$$
\nabla N_0 = -(g_1 + g_2 + g_3)
$$

即：

- $\nabla N_1 = D_m^{-T} [1, 0, 0]^T$
- $\nabla N_2 = D_m^{-T} [0, 1, 0]^T$
- $\nabla N_3 = D_m^{-T} [0, 0, 1]^T$
- $\nabla N_0 = D_m^{-T} [-1, -1, -1]^T$

**Partition of unity 推论**：$\sum_{a=0}^{3} \nabla N_a = 0$。几何意义：如果所有顶点做相同的平移（rigid translation），$F$ 不变，弹性力不产生净力。这可以作为实现的 sanity check。

#### 从节点力到能量梯度

节点力是能量梯度的负值。在代码中，梯度向量的对应位置：

```cpp
// gradient 的 += 模式
gradient.segment<3>(3 * v[a]) += V_e * P * grad_N[a];  // 注意正号：gradient = -force
```

$v_0$ 的梯度不需要单独算——利用 $\nabla N_0 = -(\nabla N_1 + \nabla N_2 + \nabla N_3)$：

```cpp
Eigen::Vector3d g0 = V_e * P * grad_N[0];  // = -V_e * P * (g1+g2+g3)
```

或者等价地，先算 $v_1, v_2, v_3$ 的贡献，$v_0$ 的贡献就是前三个的负和。

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

### 关于 Triplet 和稀疏 Hessian 装配

全局 Hessian 是 $3n \times 3n$ 的矩阵，但绝大部分元素为零（只有同一个 tet 内的顶点对之间有耦合），所以用稀疏矩阵存储。Eigen 的稀疏矩阵构建分两步：

**Step 1: 收集 Triplets**

`Eigen::Triplet<double>` 是一个 `(row, col, value)` 三元组。各能量模块把自己的贡献以 triplet 形式 append 到同一个 vector：

```cpp
// 惯性能：只有对角元素
for (int i = 0; i < n; ++i)
    triplets.emplace_back(i, i, mass_diag[i] / (dt*dt));

// 弹性能：每个 tet 贡献 12×12 = 144 个 triplet
for (int a = 0; a < 4; ++a)
    for (int b = 0; b < 4; ++b)
        for (int di = 0; di < 3; ++di)
            for (int dj = 0; dj < 3; ++dj)
                triplets.emplace_back(3*v[a]+di, 3*v[b]+dj, local_H(3*a+di, 3*b+dj));
```

同一个 `(row, col)` 可以出现多次（比如两个 tet 共享顶点），不需要手动合并。

**Step 2: 一次性构建稀疏矩阵**

```cpp
Eigen::SparseMatrix<double> H(n, n);
H.setFromTriplets(triplets.begin(), triplets.end());  // 自动合并重复位置（相加）
```

`setFromTriplets` 内部排序 + 合并同位置的值，构建压缩列存储（CSC），之后传给 `SimplicialLDLT` 求解。

**为什么不直接往稀疏矩阵插入**：直接 `coeffRef(i,j) += value` 每次都要在压缩格式里查找位置，性能极差。triplet 模式是"先全部攒好，最后一次性建"，是 Eigen 稀疏矩阵的标准用法（参考 Lec 2 §3.5-3.7）。

所以所有 `compute_hessian_triplets` 接口都设计为 `std::vector<Triplet<double>>&`（`+=` 模式）：各能量模块往同一个 vector 追加贡献，solver 统一建矩阵。GravityEnergy 不贡献任何 triplet（线性能量，Hessian 为零）。

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

## File 3: `energy/tet_material_model.hpp`

材料模型的 concept 定义。

### 设计思路：concept + 模板，不用虚函数也不用 CRTP

`compute_energy` / `compute_pk1` / `compute_hessian` 在每次 Newton 迭代中被**每个 tet** 调用——最热的内循环。需要编译器能 inline。

三种方案对比：

| 方案 | inline | 侵入性 | 错误提示 |
|------|--------|--------|----------|
| 虚函数 | 不能 | 需要继承基类 | 运行时才发现接口不匹配 |
| CRTP | 能 | 需要继承 `Base<Derived>` | 编译期，但错误信息难读 |
| **concept** | 能 | **零侵入**——材料就是普通 struct | 编译期，错误信息清晰指向缺了哪个方法 |

C++23 项目首选 concept：材料类型不需要继承任何基类，只要满足接口约束即可。编译器既能 inline，又能在约束不满足时给出清晰的错误提示。

如果未来需要运行时选材料（编辑器下拉菜单），可以用 `std::variant<FixedCorotatedMaterial, NeoHookeanMaterial, ...>` + `std::visit`，内层仍保持模板 inline。

```cpp
namespace rtr::system::physics::ipc {

/// MaterialModel concept：任何满足此约束的类型都可以作为材料模型
template <typename M>
concept MaterialModel = requires(const M& m, const Eigen::Matrix3d& F,
                                  double vol, double E, double nu) {
    // 弹性能密度 × rest_volume
    { m.compute_energy(F, vol, E, nu) } -> std::convertible_to<double>;
    // PK1 stress P = dΨ/dF (3×3)
    { m.compute_pk1(F, vol, E, nu) } -> std::convertible_to<Eigen::Matrix3d>;
    // d²Ψ/dF² (9×9)
    { m.compute_hessian(F, vol, E, nu) } -> std::convertible_to<Eigen::Matrix<double, 9, 9>>;
};

}
```

这样任何提供 `compute_energy` / `compute_pk1` / `compute_hessian` 三个 const 方法的 struct 自动满足 `MaterialModel`，不需要继承、不需要注册。

### F 的计算

给定 tet `[v0, v1, v2, v3]`，当前位置 `x`：

```cpp
Ds.col(0) = x[v1] - x[v0];
Ds.col(1) = x[v2] - x[v0];
Ds.col(2) = x[v3] - x[v0];
F = Ds * Dm_inv;
```

这个计算放在装配层（`MaterialEnergy`），不放在材料模型里。材料模型只关心"给我一个 $F$，我告诉你能量/应力/Hessian"。

### 从 F 的导数到 DOF 的导数

材料模型返回对 `F` 的 PK1 stress (3×3) 和 Hessian (9×9)。

装配层负责通过链式法则映射到全局 DOF：

```text
dE/dx_a = V * P * ∇N_a                               // 见 Step 3-3.5 推导
d²E/dx_a dx_b = V * (∂F/∂x_a)ᵀ * (d²Ψ/dF²) * (∂F/∂x_b)
```

其中 $\partial F / \partial x$ 由 $D_m^{-1}$ 决定，是常数矩阵。

---

## File 4: `energy/material_model/tet_fixed_corotated.hpp`

Day 1 唯一 concrete material。一个普通 struct，自动满足 `TetMaterialModel` concept。

```cpp
namespace rtr::system::physics::ipc {

struct FixedCorotatedMaterial {
    double compute_energy(
        const Eigen::Matrix3d& F, double rest_volume,
        double youngs_modulus, double poisson_ratio) const;

    Eigen::Matrix3d compute_pk1(
        const Eigen::Matrix3d& F, double rest_volume,
        double youngs_modulus, double poisson_ratio) const;

    Eigen::Matrix<double, 9, 9> compute_hessian(
        const Eigen::Matrix3d& F, double rest_volume,
        double youngs_modulus, double poisson_ratio) const;
};

static_assert(TetMaterialModel<FixedCorotatedMaterial>);

}
```

### 前置：Lame 参数

**参考：Lec 7 §5.5**

工程中常用 Young's modulus $E$（刚度）和 Poisson ratio $\nu$（不可压程度）来描述材料。FEM 公式需要 Lame 参数：

$$
\mu = \frac{E}{2(1+\nu)}, \qquad \lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}
$$

物理含义：
- $\mu$（剪切模量）：抵抗形状变化（剪切/扭转）的能力
- $\lambda$：抵抗体积变化的能力（$\nu \to 0.5$ 时 $\lambda \to \infty$，趋近不可压）

**代码**：`compute_lame_parameters(youngs_modulus, poisson_ratio)` → `LameParameters{mu, lambda}`

### 前置：SVD 与极分解

**参考：Lec 7 §3**

Fixed corotated 的核心工具是**极分解** $F = RS$，把变形分解为旋转 $R$ + 拉伸 $S$。

实际计算通过 SVD 完成：

$$
F = U \Sigma V^T
$$

其中 $U, V$ 是正交矩阵，$\Sigma = \text{diag}(\sigma_1, \sigma_2, \sigma_3)$ 是奇异值（$\sigma_i \geq 0$）。

旋转部分：

$$
R = U V^T
$$

但 SVD 不保证 $\det(UV^T) = +1$（可能是 $-1$，即反射）。需要修正：

```
如果 det(U * V^T) < 0:
    翻转 U 的最后一列：U[:, 2] *= -1
    翻转对应的奇异值：σ₂ *= -1
    此时 det(U * V^T) = +1，R 是真正的旋转
```

修正后的奇异值变成"带符号奇异值" $\hat{\sigma}$（可能有一个为负）。这很重要——它让极分解在 $J < 0$（翻转的 tet）时也有定义。

**代码**：`compute_rotation_data(F)` → `RotationData{UD, V, R, signed_singular_values}`

其中 `UD = U * D`（D 是符号修正对角矩阵，`D = diag(1, 1, ±1)`）。保存 `UD` 和 `V` 是因为 Hessian 计算中需要它们。

### Energy: $\Psi(F)$

**参考：Lec 7 §5.2**

Fixed corotated 能量密度：

$$
\Psi(F) = \mu \|F - R\|_F^2 + \frac{\lambda}{2}(J - 1)^2
$$

- **第一项** $\mu\|F - R\|_F^2$：惩罚变形偏离纯旋转的程度。$F = R$ 时为 0（纯旋转不花能量，rigid null space）
- **第二项** $\frac{\lambda}{2}(J-1)^2$：惩罚体积变化。$J = \det(F) = 1$ 时为 0（体积不变不花能量）

总能量 = 能量密度 × rest volume：

$$
E = V_e \cdot \Psi(F)
$$

**用 SVD 展开第一项**（证明它只依赖奇异值）：

$$
\|F - R\|_F^2 = \|U\hat{\Sigma}V^T - UDV^T\|_F^2 = \|U(\hat{\Sigma} - D)V^T\|_F^2 = \|\hat{\Sigma} - D\|_F^2
$$

最后一步因为 Frobenius 范数旋转不变。展开：

$$
= (\hat{\sigma}_1 - 1)^2 + (\hat{\sigma}_2 - 1)^2 + (\hat{\sigma}_3 - d_3)^2
$$

其中 $d_3 = \pm 1$（D 的符号修正）。所以第一项度量的是"每个主轴方向的拉伸比偏离 1 的程度"。

**代码**：

```cpp
double compute_energy(...) const {
    auto lame = compute_lame_parameters(youngs_modulus, poisson_ratio);
    auto rotation_data = compute_rotation_data(F);
    double J = F.determinant();
    double density = lame.mu * (F - rotation_data.R).squaredNorm()
                   + 0.5 * lame.lambda * std::pow(J - 1.0, 2.0);
    return rest_volume * density;
}
```

`(F - R).squaredNorm()` 就是 $\|F - R\|_F^2$，Eigen 直接提供。

**验证**：$F = R$（纯旋转）→ $\Psi = 0 + \frac{\lambda}{2}(1-1)^2 = 0$ ✓

### PK1 Stress: $P = \partial\Psi / \partial F$

**参考：Lec 8 §2**

需要对 $\Psi = \mu\|F-R\|_F^2 + \frac{\lambda}{2}(J-1)^2$ 关于 $F$ 求导。分两项求导。

#### 第一项：$\frac{\partial}{\partial F}\|F - R\|_F^2$

表面上看 $R$ 也依赖 $F$（通过 SVD），似乎需要求 $\partial R / \partial F$。但有一个关键事实：

**定理**：$\frac{\partial}{\partial F}\|F - R(F)\|_F^2 = 2(F - R)$

即可以"当 $R$ 是常数"来求导。这是因为：
- $\|F-R\|_F^2 = \text{tr}((F-R)^T(F-R)) = \|F\|_F^2 - 2\text{tr}(R^TF) + \|R\|_F^2$
- $\|R\|_F^2 = 3$（旋转矩阵的性质）
- $\text{tr}(R^TF)$ 是 $F$ 的奇异值之和（$\sum \hat{\sigma}_i$），可以证明其关于 $F$ 的导数恰好是 $R$
- 所以 $\partial/\partial F = 2F - 2R = 2(F-R)$

直觉：$R(F)$ 是 $F$ 在旋转群 $SO(3)$ 上的最近点投影。投影点的变化方向总是沿"约束面"（旋转群的切空间），与 $(F-R)$ 正交，所以对导数没有贡献。

#### 第二项：$\frac{\partial}{\partial F}\frac{\lambda}{2}(J-1)^2$

用链式法则：$\lambda(J-1) \cdot \frac{\partial J}{\partial F}$

**$\frac{\partial J}{\partial F}$ 是什么？**

$J = \det(F)$ 对 $F$ 的导数是 **cofactor matrix**（余子式矩阵）：

$$
\frac{\partial J}{\partial F} = \text{cof}(F)
$$

等价于 $J \cdot F^{-T}$（当 $F$ 可逆时），但 cofactor 的形式在 $F$ 奇异时也有定义，数值更稳定。

**Cofactor 矩阵的几何意义**：

3D 矩阵 $F$ 的 cofactor 矩阵的三列分别是另外两列的叉积：

$$
\text{cof}(F) = \begin{bmatrix} F_1 \times F_2, & F_2 \times F_0, & F_0 \times F_1 \end{bmatrix}
$$

其中 $F_i = F[:,i]$ 是 $F$ 的第 $i$ 列。

**代码**：`compute_cofactor_matrix(F)` 直接用叉积计算。

**验证**：$\text{cof}(F) : F = \sum_{ij} \text{cof}(F)_{ij} F_{ij} = 3 \det(F) = 3J$（这是 Euler 恒等式的推论）。

#### 合并两项

$$
\boxed{P = 2\mu(F - R) + \lambda(J - 1)\,\text{cof}(F)}
$$

**代码**：

```cpp
Eigen::Matrix3d compute_pk1(...) const {
    auto lame = compute_lame_parameters(youngs_modulus, poisson_ratio);
    auto rotation_data = compute_rotation_data(F);
    double J = F.determinant();
    Eigen::Matrix3d cofactor = compute_cofactor_matrix(F);
    return 2.0 * lame.mu * (F - rotation_data.R) + lame.lambda * (J - 1.0) * cofactor;
}
```

注意 `compute_pk1` 返回的是 $\partial\Psi/\partial F$（不含 $V_e$），$V_e$ 在装配层 `MaterialEnergy` 中乘入。

**验证**：
- $F = I$（无变形）：$R = I$，$J = 1$，$P = 2\mu(I-I) + \lambda(1-1)\text{cof}(I) = 0$ ✓
- $F = 2I$（均匀膨胀 2 倍）：$R = I$，$J = 8$，$P = 2\mu(2I-I) + \lambda \cdot 7 \cdot 4I = 2\mu I + 28\lambda I$。
  体积变大，应力是正的（抵抗膨胀），符合直觉 ✓

### Hessian: $\partial P / \partial F$（9×9 矩阵）

**参考：Lec 8 §4-5**

Hessian 是 PK1 stress 对 $F$ 的二阶导数，形状为 9×9（把 3×3 的 $F$ 和 $P$ 都展平为 9 维向量）。

$$
\left(\frac{\partial^2 \Psi}{\partial F^2}\right)_{ij} = \frac{\partial P_i}{\partial F_j}, \qquad i, j \in \{1, \ldots, 9\}
$$

对 $P = 2\mu(F - R) + \lambda(J-1)\text{cof}(F)$，取微分 $dP$（即给 $F$ 一个微小扰动 $dF$，看 $P$ 怎么变）：

$$
dP = 2\mu(dF - dR) + \lambda\big(dJ \cdot \text{cof}(F) + (J-1) \cdot d\text{cof}(F)\big)
$$

三个需要计算的微分量：$dR$, $dJ$, $d\text{cof}(F)$。

#### $dJ$：行列式的微分

$$
dJ = \text{cof}(F) : dF = \sum_{ij} \text{cof}(F)_{ij} \, dF_{ij}
$$

这是行列式微分的标准公式。直觉：行列式是三列围成的平行六面体体积，微分就是把每列的变化投影到对面两列张成的法向量上（= cofactor）。

**代码**：`double dJ = cofactor.cwiseProduct(dF).sum()`

#### $d\text{cof}(F)$：cofactor 矩阵的微分

对 $\text{cof}(F) = [F_1 \times F_2, \; F_2 \times F_0, \; F_0 \times F_1]$ 用乘法法则求微分：

$$
d\text{cof}(F) = \begin{bmatrix}
dF_1 \times F_2 + F_1 \times dF_2, &
dF_2 \times F_0 + F_2 \times dF_0, &
dF_0 \times F_1 + F_0 \times dF_1
\end{bmatrix}
$$

**代码**：`differential_cofactor_matrix(F, dF)`

```cpp
inline Eigen::Matrix3d differential_cofactor_matrix(const Matrix3d& F, const Matrix3d& dF) {
    Matrix3d result;
    result.col(0) = dF.col(1).cross(F.col(2)) + F.col(1).cross(dF.col(2));
    result.col(1) = dF.col(2).cross(F.col(0)) + F.col(2).cross(dF.col(0));
    result.col(2) = dF.col(0).cross(F.col(1)) + F.col(0).cross(dF.col(1));
    return result;
}
```

每一列是叉积的乘法法则展开，对应 cofactor 定义中的两项。

#### $dR$：旋转矩阵的微分（最复杂的部分）

**参考：Lec 8 §4.3，Smith et al. 2019 "Analytic Eigensystems for Isotropic Distortion Energies"**

这是整个 Hessian 推导中最非 trivial 的部分。$R = UDV^T$ 依赖 SVD，而 SVD 的导数涉及奇异值之间的耦合。

**推导思路**：

$R$ 是正交矩阵，所以 $dR$ 一定是 $R$ 乘以一个反对称矩阵 $\hat{\omega}$：

$$
dR = R \cdot \hat{\omega}
$$

（正交矩阵的切空间是反对称矩阵。$R + dR$ 近似正交 $\Leftrightarrow$ $(R+dR)^T(R+dR) \approx I$ $\Leftrightarrow$ $R^T dR + dR^T R \approx 0$ $\Leftrightarrow$ $R^T dR$ 反对称。）

**求 $\hat{\omega}$**：

从 $R^T F = D\hat{\Sigma}$（对角矩阵，SVD 的性质）出发，对两边求微分：

$$
dR^T F + R^T dF = d(D\hat{\Sigma})
$$

左边代入 $dR = R\hat{\omega}$：

$$
\hat{\omega}^T R^T F + R^T dF = d(D\hat{\Sigma})
$$

即 $-\hat{\omega} D\hat{\Sigma} + R^T dF = d(D\hat{\Sigma})$

定义 $M = R^T dF - dF^T R$（注意 $M$ 是反对称的），以及 $B = V^T M V$。

在 $V$ 基下，$\hat{\omega}$ 在 $V$ 基下的表示 $\tilde{\omega} = V^T \hat{\omega} V$ 的上三角元素为：

$$
\tilde{\omega}_{ij} = \frac{B_{ij}}{\hat{\sigma}_i + \hat{\sigma}_j}, \qquad i < j
$$

最终：

$$
dR = UD \cdot \tilde{\omega} \cdot V^T
$$

**代码**：`differential_rotation(dF, rotation_data)`

```cpp
inline Matrix3d differential_rotation(const Matrix3d& dF, const RotationData& rot) {
    // M = R^T dF - dF^T R  (反对称矩阵)
    Matrix3d M = rot.R.transpose() * dF - dF.transpose() * rot.R;
    // 变换到 V 基下
    Matrix3d B = rot.V.transpose() * M * rot.V;
    // 解 ω
    Matrix3d omega_hat = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            double denom = rot.signed_singular_values[i]
                         + rot.signed_singular_values[j];
            if (std::abs(denom) <= 1e-12) continue;  // 退化保护
            double omega_ij = B(i, j) / denom;
            omega_hat(i, j) = omega_ij;
            omega_hat(j, i) = -omega_ij;       // 反对称
        }
    // dR = UD * ω * V^T
    return rot.UD * omega_hat * rot.V.transpose();
}
```

**退化保护**：当 $\hat{\sigma}_i + \hat{\sigma}_j \approx 0$ 时，对应的旋转模态不确定（类似除以零）。这发生在"几乎完全压扁"的 tet（$\sigma \approx 0$）。用 `continue`（视为 0）是安全的——此模态的贡献很小。

#### 组装 9×9 Hessian

有了 $dR$, $dJ$, $d\text{cof}(F)$ 的计算方法，Hessian 的组装策略是：**逐列构建**。

对 $j = 0, 1, \ldots, 8$（$F$ 展平后的 9 个分量），令 $dF = e_j$（第 $j$ 个标准基的 3×3 reshape），算出 $dP$，展平后就是 Hessian 的第 $j$ 列。

**代码**：

```cpp
Eigen::Matrix<double, 9, 9> compute_hessian(...) const {
    auto lame = compute_lame_parameters(youngs_modulus, poisson_ratio);
    auto rotation_data = compute_rotation_data(F);
    double J = F.determinant();
    Matrix3d cofactor = compute_cofactor_matrix(F);

    Matrix<double, 9, 9> hessian = Matrix<double, 9, 9>::Zero();

    for (int j = 0; j < 9; ++j) {
        // dF = 第 j 个方向的基向量（reshape 为 3×3）
        Matrix3d dF = Matrix3d::Zero();
        Map<Matrix<double, 9, 1>>(dF.data())[j] = 1.0;

        // 三个微分量
        Matrix3d dR = differential_rotation(dF, rotation_data);
        Matrix3d dCof = differential_cofactor_matrix(F, dF);
        double dJ = cofactor.cwiseProduct(dF).sum();

        // dP = 2μ(dF - dR) + λ(dJ·cof(F) + (J-1)·dcof(F))
        Matrix3d dP = 2.0 * lame.mu * (dF - dR)
                     + lame.lambda * (dJ * cofactor + (J - 1.0) * dCof);

        // 展平 dP 为 9 维列向量
        hessian.col(j) = flatten_matrix(dP);
    }

    return hessian;
}
```

**为什么逐列构建而非写闭式表达式？**

闭式 9×9 Hessian 公式存在（见 Lec 8 §4.5 的 A/B block 分解），但在 diagonal space 操作后再旋转回世界系，代码很复杂且容易出错。逐列构建方式：

1. 代码直接对应 $dP$ 的物理定义——容易验证
2. 复用已有的 `differential_rotation` 和 `differential_cofactor_matrix` 函数
3. 9 次循环的开销对 Day 1 完全可接受（每个 tet 9 次 3×3 操作 ≈ 几百次浮点运算）
4. 方便用 FD 验证（对某一列单独验证 = 固定 $dF$ 方向的 FD）

#### 数值例子

取 $F = \text{diag}(2, 1, 0.5)$（x 方向拉伸 2 倍，z 方向压缩一半），$\mu = 1$，$\lambda = 1$。

**SVD**：$U = V = I$（已经是对角阵），$\hat{\sigma} = (2, 1, 0.5)$，$R = I$，$J = 1.0$

**Energy**：$\Psi = 1 \cdot ((2-1)^2 + (1-1)^2 + (0.5-1)^2) + \frac{1}{2}(1-1)^2 = 1 + 0 + 0.25 = 1.25$

**PK1**：$J = 1$ 所以第二项消失。$P = 2(F - I) = 2 \cdot \text{diag}(1, 0, -0.5) = \text{diag}(2, 0, -1)$

物理解读：x 方向被拉伸 → 正应力（回弹力），z 方向被压缩 → 负应力（也是回弹力），y 方向无变形 → 零应力。✓

**Hessian 的第 0 列**（$dF$ 只在 $F_{00}$ 方向）：

$dF = e_{00}$（只有 (0,0) 位置为 1 的 3×3 矩阵）

- $R = I$，所以 $M = I^T dF - dF^T I = dF - dF^T = 0$（因为 $dF$ 是对称的 $e_{00}$）
- $\hat{\omega} = 0$，$dR = 0$
- $\text{cof}(F) = \text{diag}(0.5, 1, 2)$（对角阵的 cofactor），$dJ = \text{cof}(F)_{00} \cdot 1 = 0.5$
- $d\text{cof}(F)$：$dF$ 只有第 0 列的第 0 个元素为 1，第 0 列 cross 第 1 列在 cofactor 中的贡献...对对角 $F$ 可以手算得 $d\text{cof}(F) = \text{diag}(0, 0.5, 1)$
- $dP = 2(dF - 0) + 1 \cdot (0.5 \cdot \text{diag}(0.5, 1, 2) + 0 \cdot d\text{cof}) = \text{diag}(2, 0, 0) + \text{diag}(0.25, 0.5, 1) = \text{diag}(2.25, 0.5, 1)$

所以 Hessian 第 0 列的 (0,0), (4,0), (8,0) 位置分别是 2.25, 0.5, 1。其余为零（因为对角 $F$ 各方向解耦）。

### PSD Projection

**参考：Lec 8 §4.6**

Newton 法要求 Hessian 正半定（PSD），否则搜索方向可能不是下降方向。但弹性能的 Hessian 在大变形下可能有负特征值（尤其是压缩方向）。

处理策略：对 9×9 的 $\partial^2\Psi/\partial F^2$ 做特征分解，把负特征值 clamp 到 0：

$$
H = Q \Lambda Q^T \quad \Rightarrow \quad H_{PSD} = Q \max(\Lambda, 0) Q^T
$$

**Day 1 简化**：先用全局对角 regularization $H + \epsilon I$（$\epsilon$ 较小时效果类似），如果出现 solver 不收敛再加真正的 per-element PSD projection。在代码中标注 `// TODO: per-element PSD projection`。

验收：
- 单 tet 在 $F = I$ 时 energy = 0
- 小扰动下 PK1 与 FD gradient 一致（tolerance ~$10^{-5}$）
- Hessian 与 FD Hessian 一致（逐列对比 = FD PK1）
- 压缩/拉伸方向的能量变化符合物理直觉

---

## File 5: `energy/material_energy.hpp`

将 `TetMaterialModel` concept 对齐到全局 Energy 接口（`compute_energy` / `compute_gradient` / `compute_hessian_triplets` 操作全局 DOF 向量）。这个模板类是材料模型和 solver 之间的桥梁：

- **输入**：任何满足 `TetMaterialModel` concept 的材料 + `TetBody` + 全局 `x`
- **输出**：全局 energy（scalar）、gradient（VectorXd）、Hessian triplets

```cpp
namespace rtr::system::physics::ipc {

template <TetMaterialModel Material>
struct MaterialEnergy {
    struct Input {
        const TetBody& body;
        const Eigen::VectorXd& x;
        const Material& material;
    };

    static double compute_energy(const Input& input);
    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient);       // += 模式
    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets); // += 模式
};

}
```

### 为什么叫 `MaterialEnergy` 而非 `TetElasticAssembler`

这个类的角色是：**把任意 `TetMaterialModel` 适配成统一的 Energy 接口**。名字体现它是 Energy 体系的一部分（和 `InertialEnergy`、`GravityEnergy` 平级），而不只是一个装配工具。

调用方看到的是统一的 energy 模式：

```cpp
// IPCSystem::compute_total_energy()
E += InertialEnergy::compute_energy(x, x_hat, mass_diag, dt);
E += GravityEnergy::compute_energy(x, mass_diag, gravity);
E += MaterialEnergy<FixedCorotatedMaterial>::compute_energy({body, x, m_material});
//   ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ concept 约束，编译期 inline
```

### 理论基础：从单元局部量到全局 DOF 的桥接

材料模型在 $F$ 空间工作（3×3 矩阵），而 solver 在全局 DOF 空间工作（$3n$ 维向量）。`MaterialEnergy` 的核心任务就是做这两个空间之间的转换。

转换涉及三层链式法则：

$$
\underbrace{E(x)}_{\text{global DOF}} = \sum_e \underbrace{V_e \cdot \Psi(F_e)}_{\text{per-tet energy}}, \quad F_e = \underbrace{D_s(x) \cdot D_m^{-1}}_{\text{x → F 映射}}
$$

---

### 前置工具函数

#### `build_Ds`：从全局 DOF 提取当前构型边向量

给定 tet $[v_0, v_1, v_2, v_3]$ 和全局位置向量 $x$：

$$
D_s = [x_{v_1} - x_{v_0}, \quad x_{v_2} - x_{v_0}, \quad x_{v_3} - x_{v_0}]
$$

**代码**：

```cpp
inline Matrix3d build_Ds(const VectorXd& x, size_t dof_offset, const array<size_t, 4>& tet) {
    Vector3d x0 = x.segment<3>(dof_offset + 3 * tet[0]);
    Matrix3d Ds;
    Ds.col(0) = x.segment<3>(dof_offset + 3 * tet[1]) - x0;
    Ds.col(1) = x.segment<3>(dof_offset + 3 * tet[2]) - x0;
    Ds.col(2) = x.segment<3>(dof_offset + 3 * tet[3]) - x0;
    return Ds;
}
```

`dof_offset` 是 body 在全局 DOF 向量中的起始位置（多 body 时，第二个 body 的顶点从 `3 * vertex_count_of_body_0` 开始）。

#### `compute_shape_gradients`：计算 $\nabla N_a$

回顾前文推导：$\nabla N_a$ 是 shape function 对参考坐标的梯度，由 $D_m^{-1}$ 决定。

$$
\nabla N_1 = D_m^{-T} e_0, \quad \nabla N_2 = D_m^{-T} e_1, \quad \nabla N_3 = D_m^{-T} e_2
$$

$$
\nabla N_0 = -(\nabla N_1 + \nabla N_2 + \nabla N_3)
$$

**代码**：

```cpp
inline array<Vector3d, 4> compute_shape_gradients(const Matrix3d& Dm_inv) {
    Matrix3d G = Dm_inv.transpose();  // G = Dm_inv^T
    array<Vector3d, 4> gradients;
    gradients[1] = G.col(0);          // ∇N_1 = G 的第 0 列
    gradients[2] = G.col(1);          // ∇N_2 = G 的第 1 列
    gradients[3] = G.col(2);          // ∇N_3 = G 的第 2 列
    gradients[0] = -(gradients[1] + gradients[2] + gradients[3]);  // partition of unity
    return gradients;
}
```

注意返回 4 个梯度（包括 $v_0$），不是 3 个。$v_0$ 的梯度通过 partition of unity 隐式得到。

#### `build_dFdx_matrix`：$\partial F / \partial x_a$ 的矩阵表示

这是从 DOF 空间到 $F$ 空间的 Jacobian。前文推导了：

$$
\delta F = \delta x_a \otimes \nabla N_a
$$

其中 $\delta x_a$ 是 3D 向量，$\delta F$ 是 3×3 矩阵。如果把 $F$ 展平成 9 维列向量（Eigen 的列优先序：$\text{vec}(F) = [F_{00}, F_{10}, F_{20}, F_{01}, F_{11}, F_{21}, F_{02}, F_{12}, F_{22}]^T$），则：

$$
\text{vec}(\delta F) = \underbrace{\frac{\partial \text{vec}(F)}{\partial x_a}}_{9 \times 3 \text{ 矩阵}} \cdot \delta x_a
$$

**推导 9×3 矩阵的结构**：

$(\delta F)_{ij} = (\delta x_a)_i \cdot (\nabla N_a)_j$，即 $\delta F$ 的第 $(i,j)$ 元素 = $\delta x_a$ 的第 $i$ 分量 × $\nabla N_a$ 的第 $j$ 分量。

在 Eigen 列优先展平下，$F_{ij}$ 对应 $\text{vec}(F)$ 的第 $i + 3j$ 个位置。所以：

$$
\text{vec}(\delta F)_{i+3j} = (\nabla N_a)_j \cdot (\delta x_a)_i
$$

写成矩阵形式：$\text{dFdx}$ 的第 $(i+3j, i)$ 元素 = $(\nabla N_a)_j$。

**用数值例子验证**：$\nabla N_1 = (g_0, g_1, g_2)$

$$
\text{dFdx}_1 = \begin{bmatrix}
g_0 & 0 & 0 \\
0 & g_0 & 0 \\
0 & 0 & g_0 \\
g_1 & 0 & 0 \\
0 & g_1 & 0 \\
0 & 0 & g_1 \\
g_2 & 0 & 0 \\
0 & g_2 & 0 \\
0 & 0 & g_2
\end{bmatrix}
$$

每 3 行一组，对应 $F$ 的一列。每组内是 $(\nabla N_a)_j$ 乘以 $I_3$。

**代码**：

```cpp
inline Matrix<double, 9, 3> build_dFdx_matrix(const Vector3d& grad_N) {
    Matrix<double, 9, 3> dFdx = Matrix<double, 9, 3>::Zero();
    for (int col = 0; col < 3; ++col) {        // F 的列 index j
        for (int row = 0; row < 3; ++row) {     // F 的行 index i = δx_a 的分量 index
            dFdx(row + 3 * col, row) = grad_N[col];
            //   ^^^^^^^^^^^^^  ^^^
            //   vec(F) index   δx_a 分量
        }
    }
    return dFdx;
}
```

**验证**：$\nabla N_a = (1, 0, 0)$（参考 tet $D_m = I$ 时的 $\nabla N_1$）时，$\text{dFdx}$ 的非零位仅在 $(0,0), (1,1), (2,2)$，值为 1。这意味着移动 $x_1$ 只改变 $F$ 的第 0 列——正确，因为 $D_s$ 的第 0 列 = $x_1 - x_0$。✓

---

### Energy 装配

**理论**：

$$
E_{elastic}(x) = \sum_{e} V_e \cdot \Psi(F_e), \quad F_e = D_s^{(e)} \cdot D_m^{(e)-1}
$$

直接遍历所有 tet，逐个算 $F$，调材料的 `compute_energy`，累加即可。

**代码**：

```cpp
static double compute_energy(const Input& input) {
    double total = 0.0;
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        const auto& tet = input.body.geometry.tets[t];
        Matrix3d Ds = build_Ds(input.x, input.body.info.dof_offset, tet);
        Matrix3d F = Ds * input.body.geometry.Dm_inv[t];
        total += input.material.compute_energy(
            F, input.body.geometry.rest_volumes[t],
            input.body.youngs_modulus, input.body.poisson_ratio
        );
    }
    return total;
}
```

注意 `compute_energy` 返回的已经是 $V_e \cdot \Psi(F)$（rest_volume 在材料内部乘入）。

---

### Gradient 装配

**理论**：

回顾前文 Step 3-3.5 的推导结论：

$$
\frac{\partial E_e}{\partial x_a} = V_e \cdot P_e \cdot \nabla N_a
$$

这是一个 3D 向量（节点 $a$ 的 3 个 DOF 分量各一个梯度值）。

装配步骤：

1. 对每个 tet $e$，计算 $F_e$，调材料得 $P_e$（3×3）
2. 计算 4 个 $\nabla N_a$
3. 对 $a = 0, 1, 2, 3$：$\text{gradient}[3 v_a : 3 v_a + 3] \mathrel{+}= V_e \cdot P_e \cdot \nabla N_a$

多个 tet 共享顶点时，梯度自然累加（`+=` 模式）。

**代码**：

```cpp
static void compute_gradient(const Input& input, VectorXd& gradient) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        const auto& tet = input.body.geometry.tets[t];
        Matrix3d Ds = build_Ds(input.x, input.body.info.dof_offset, tet);
        Matrix3d F = Ds * input.body.geometry.Dm_inv[t];
        double V = input.body.geometry.rest_volumes[t];

        // 材料给出 P = ∂Ψ/∂F (3×3)
        Matrix3d P = input.material.compute_pk1(F, V,
            input.body.youngs_modulus, input.body.poisson_ratio);

        // ∇N_a：4 个 shape function 梯度
        auto grad_N = compute_shape_gradients(input.body.geometry.Dm_inv[t]);

        // 对 4 个顶点分别累加
        for (int a = 0; a < 4; ++a) {
            Vector3d local_grad = V * P * grad_N[a];   // 3D 向量
            Index base = input.body.info.dof_offset + 3 * tet[a];
            gradient.segment<3>(base) += local_grad;
        }
    }
}
```

**关键理解**：`P * grad_N[a]` 是矩阵乘向量（3×3 × 3×1 = 3×1）。$P$ 编码了"每个参考方向的应力"，$\nabla N_a$ 编码了"节点 $a$ 影响的参考方向"。两者相乘 = "节点 $a$ 从应力场中分到的力"。

**数值验证**（参考 tet $D_m = I$，$F = \text{diag}(2,1,1)$，$\mu=1, \lambda=1, V_e = 1/6$）：

$P = \text{diag}(2, 0, -1)$（前文推导），$\nabla N_1 = (1, 0, 0)$

$\text{grad}_{v_1} = \frac{1}{6} \cdot \text{diag}(2,0,-1) \cdot (1,0,0)^T = \frac{1}{6}(2, 0, 0)^T$

$v_1$ 沿 x 正方向收到梯度（能量关于 $x_1$ 的 x 分量的导数为正 → 继续拉伸会增加能量 → 梯度方向 = 能量增加方向）。✓

$\nabla N_0 = -(1,0,0) - (0,1,0) - (0,0,1) = (-1,-1,-1)$

$\text{grad}_{v_0} = \frac{1}{6} \cdot \text{diag}(2,0,-1) \cdot (-1,-1,-1)^T = \frac{1}{6}(-2, 0, 1)^T$

$v_0$ 沿 x 负方向受力、z 正方向受力。$v_0$ 是参考原点，x 方向的拉伸让 $v_1$ 远离 $v_0$，所以 $v_0$ 有"被拉向 x 正方向"的趋势——但梯度是能量对位移的导数，$v_0$ 往 x 负方向移会缩小拉伸，降能，所以梯度在 x 方向为负。✓

---

### Hessian 装配

这是 `MaterialEnergy` 最复杂的部分。需要把材料给的 9×9 的 $\partial^2\Psi/\partial F^2$ 映射到全局 DOF 空间的稀疏 Hessian。

#### 理论推导

**目标**：计算 $\frac{\partial^2 E_e}{\partial x_a \partial x_b}$——tet $e$ 对全局 Hessian 中 $(v_a, v_b)$ block 的贡献（3×3 矩阵）。

从 gradient 出发再求一次导：

$$
\frac{\partial E_e}{\partial x_a} = V_e \cdot P \cdot \nabla N_a
$$

$$
\frac{\partial^2 E_e}{\partial x_a \partial x_b} = V_e \cdot \frac{\partial (P \cdot \nabla N_a)}{\partial x_b}
$$

$\nabla N_a$ 是常数（只依赖 rest shape），所以：

$$
= V_e \cdot \frac{\partial P}{\partial x_b} \cdot \nabla N_a
$$

这里 $\frac{\partial P}{\partial x_b}$ 是"$P$（3×3 矩阵）对 $x_b$（3D 向量）的导数"——这是一个 4 阶张量，直接操作很痛苦。

**关键技巧：用展平 + 链式法则绕开 4 阶张量**

把所有矩阵展平为向量，用矩阵链式法则：

$$
\text{vec}\left(\frac{\partial E_e}{\partial x_a}\right) = V_e \cdot \underbrace{\left(\frac{\partial \text{vec}(F)}{\partial x_a}\right)^T}_{3 \times 9} \cdot \underbrace{\text{vec}(P)}_{9 \times 1}
$$

这里 $\frac{\partial \text{vec}(F)}{\partial x_a}$ 就是前面推导的 `dFdx_a`（9×3 矩阵）。

再对 $x_b$ 求导：

$$
\frac{\partial^2 E_e}{\partial x_a \partial x_b} = V_e \cdot \text{dFdx}_a^T \cdot \underbrace{\frac{\partial^2 \Psi}{\partial F^2}}_{9 \times 9} \cdot \text{dFdx}_b
$$

**这就是代码中的核心公式**：

$$
\boxed{H_{ab}^{(e)} = V_e \cdot \text{dFdx}_a^T \cdot H_F \cdot \text{dFdx}_b}
$$

其中 $H_F = \partial^2\Psi/\partial F^2$ 是材料返回的 9×9 Hessian，$\text{dFdx}_a$ 是 9×3 的 Jacobian。

**维度验证**：$(3 \times 9) \cdot (9 \times 9) \cdot (9 \times 3) = 3 \times 3$ ✓

每个 tet 有 4 个顶点，产生 $4 \times 4 = 16$ 个 3×3 block，总共 $16 \times 9 = 144$ 个 scalar triplet。

#### 为什么做对称化

材料的 `compute_hessian` 可能因数值精度返回不完全对称的 9×9 矩阵（例如逐列构建方式中 `dR` 的数值误差）。在装配前做显式对称化：

```cpp
hessian_F = 0.5 * (hessian_F + hessian_F.transpose());
```

这保证了全局 Hessian 的对称性（`SimplicialLDLT` 要求输入对称矩阵）。

#### 代码实现

```cpp
static void compute_hessian_triplets(const Input& input,
                                     vector<Triplet<double>>& triplets) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        const auto& tet = input.body.geometry.tets[t];
        Matrix3d Ds = build_Ds(input.x, input.body.info.dof_offset, tet);
        Matrix3d F = Ds * input.body.geometry.Dm_inv[t];
        double V = input.body.geometry.rest_volumes[t];

        // 材料给出 9×9 的 ∂²Ψ/∂F²
        Matrix<double, 9, 9> H_F = input.material.compute_hessian(
            F, V, input.body.youngs_modulus, input.body.poisson_ratio);
        H_F = 0.5 * (H_F + H_F.transpose());  // 显式对称化

        // 预计算 4 个 dFdx 矩阵 (9×3)
        auto grad_N = compute_shape_gradients(input.body.geometry.Dm_inv[t]);
        array<Matrix<double, 9, 3>, 4> dFdx;
        for (int a = 0; a < 4; ++a)
            dFdx[a] = build_dFdx_matrix(grad_N[a]);

        // 装配 4×4 = 16 个 3×3 block
        for (int a = 0; a < 4; ++a) {
            Index row_base = input.body.info.dof_offset + 3 * tet[a];
            for (int b = 0; b < 4; ++b) {
                Index col_base = input.body.info.dof_offset + 3 * tet[b];

                // 核心公式：H_ab = V * dFdx_a^T * H_F * dFdx_b
                Matrix3d block = V * dFdx[a].transpose() * H_F * dFdx[b];

                // scatter 3×3 block 到 triplets
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        triplets.emplace_back(row_base + i, col_base + j, block(i, j));
            }
        }
    }
}
```

#### 完整数据流

```
对每个 tet e:
    x (3n 全局 DOF)
    │
    ├── build_Ds() ──→ D_s (3×3)
    │                   │
    │                   ├── F = D_s * Dm_inv  ──→ F (3×3)
    │                   │                          │
    │                   │   ┌──────────────────────┤
    │                   │   │                      │
    │                   │   ▼                      ▼
    │                   │   material.compute_pk1   material.compute_hessian
    │                   │   │                      │
    │                   │   ▼                      ▼
    │                   │   P (3×3)                H_F (9×9)
    │                   │   │                      │
    ├── compute_shape_gradients() ──→ ∇N_a (4 个 3D 向量)
    │                   │   │                      │
    │                   │   │      build_dFdx_matrix()
    │                   │   │              │
    │                   │   │              ▼
    │                   │   │      dFdx_a (4 个 9×3)
    │                   │   │              │
    │                   │   ▼              ▼
    │                   │   grad[3va:3va+3]    H_ab = V * dFdx_a^T * H_F * dFdx_b
    │                   │   += V * P * ∇N_a    │
    │                   │                      ▼
    │                   │              triplets.emplace_back(...)
    │                   │              (16 blocks × 9 entries = 144 triplets per tet)
```

#### 数值例子

参考 tet（$D_m = I$），$F = I$（无变形），$\mu = 1, \lambda = 1, V_e = 1/6$。

**$\nabla N$ 和 dFdx**：

$\nabla N_1 = (1,0,0)$，对应的 $\text{dFdx}_1$ 只在 $(0,0), (1,1), (2,2)$ 处为 1，其余为零。

$\nabla N_2 = (0,1,0)$，对应的 $\text{dFdx}_2$ 只在 $(3,0), (4,1), (5,2)$ 处为 1。

**$H_F$ 在 $F = I$ 时**：

$F = I$ → $R = I$，$J = 1$，所以 $dP = 2\mu(dF - dR) + \lambda(dJ \cdot I + 0)$（$J-1=0$ 消掉了 $d\text{cof}$ 项）。

$$
H_F = 2\mu (I_9 - dR/dF) + \lambda \, \text{vec}(\text{cof}(I)) \cdot \text{vec}(\text{cof}(I))^T
$$

$\text{cof}(I) = I$，$\text{vec}(I) = [1,0,0,0,1,0,0,0,1]^T$。

这给出 $H_F = 2(I_9 - dR/dF) + \text{vec}(I)\text{vec}(I)^T$。

**Block $H_{11}$**（顶点 1 对自己的贡献）：

$$
H_{11} = \frac{1}{6} \cdot \text{dFdx}_1^T \cdot H_F \cdot \text{dFdx}_1
$$

$\text{dFdx}_1$ 只选 $H_F$ 的第 0,1,2 行和第 0,1,2 列（3×3 左上角 block）。

$H_F$ 的左上 3×3 block = $2(I_3 - dR/dF \text{对应block}) + [1,0,0]^T[1,0,0]$

在 $F = I$ 时 $dR/dF$ 的对角 block 近似为零（identity 处旋转不需要修正），所以 $\approx 2I_3 + e_0 e_0^T = \text{diag}(3, 2, 2)$。

$H_{11} \approx \frac{1}{6}\text{diag}(3, 2, 2)$

物理含义：沿 x 方向移动 $v_1$ 的刚度最大（因为 $v_1$ 在 x 方向上，拉伸模式 = $\mu + \lambda$），沿 y/z 方向是纯剪切模式（$= \mu$）。✓

### 计算复杂度

每个 tet 的装配开销：

| 操作 | 复杂度 |
|------|--------|
| `build_Ds` + `F = Ds * Dm_inv` | $O(27)$（矩阵乘法） |
| `compute_pk1`（含 SVD） | $O(100+)$（SVD 主导） |
| `compute_hessian`（9 列 × SVD 微分） | $O(900+)$ |
| `dFdx_a^T * H_F * dFdx_b`（16 个 3×3 block） | $O(16 \times 81)$ |
| Triplet 写入（144 个） | $O(144)$ |

gradient 装配比 Hessian 装配便宜一个数量级（不需要 9×9 Hessian 和 16 个矩阵乘法）。这是为什么 CG solver（只需 gradient）比 Newton（需 Hessian）每步更快的原因。

验收：
- 全局 gradient 大小 = `state.dof_count()`
- 装配后 gradient 不含 NaN
- 单 tet energy 和调 `material.compute_energy` 直接算的结果一致
- Hessian triplets 对称（$H_{ab} = H_{ba}^T$，因为 $H_F$ 对称化过）
- 单 tet 的 12×12 局部 Hessian 可以通过 FD gradient 验证
