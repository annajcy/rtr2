# Day 1 理论复习：课程笔记 → 代码实现对齐

本文档按 Day 1 各 Phase 的执行顺序组织，每一节先给出需要复习的理论（指向课程笔记具体位置），再说明这些理论如何映射到你要写的代码。

课程笔记根目录：`/Users/jinceyang/Desktop/cg_course/physics_based_simulation/`

参考代码：`solid-sim-tutorial/6_inv_free/`（Neo-Hookean FEM 完整实现）

---

## Phase 1 理论：全局 DOF 与 FEM 离散基础

### 复习内容

**Lec 10 Section 2-3**（`lec10_finite_element/lec10_course_note.md`）

核心概念：FEM 把连续域上的位移场 $x(X,t)$ 写成节点自由度的插值。这个公式的推导链路如下：

#### 为什么需要这个公式：从连续到离散

物理系统的状态是一个连续位移场 $x(X,t)$：给定参考构型中的任意材料点 $X \in \Omega^0$，它告诉你这个点在时刻 $t$ 的世界坐标。但连续函数有无穷多自由度，计算机没法直接存储和求解。

FEM 的核心思想是：**用有限个节点的坐标来代表整个连续场**。

#### 推导步骤

**Step 1: 空间离散 — 把连续域剖分成单元**

把 $\Omega^0$ 剖分成不重叠的 simplex 单元（3D 为四面体），共享节点。这样域内每一点都属于某个单元。

**Step 2: 定义形函数 $N_a(X)$**

对每个节点 $a$，定义一个 shape function $N_a(X)$，它满足两个关键性质：

- **插值性**：在节点 $a$ 处 $N_a(X_a) = 1$，在其他节点 $b \neq a$ 处 $N_a(X_b) = 0$
- **Partition of unity**：$\sum_a N_a(X) = 1$，即所有形函数在任意点处的权重之和为 1

对线性四面体单元，$N_a$ 就是重心坐标（barycentric coordinates）。例如一个四面体 $(X_0, X_1, X_2, X_3)$，任何点 $X$ 可以唯一表示为：

$$
X = \lambda_0 X_0 + \lambda_1 X_1 + \lambda_2 X_2 + \lambda_3 X_3, \quad \lambda_0 + \lambda_1 + \lambda_2 + \lambda_3 = 1
$$

此时 $N_a(X) = \lambda_a$，几何直觉就是"这个点有多靠近节点 $a$"。

**Step 3: 用形函数近似连续场**

把连续位移场 $x(X,t)$ 近似为节点坐标的加权插值：

$$
\hat{x}(X) = \sum_a x_a N_a(X)
$$

其中 $x_a \in \mathbb{R}^3$ 是节点 $a$ 的当前坐标（未知量），$N_a(X)$ 是已知的形函数。

**验证这个近似是合理的**：在节点 $b$ 处，$\hat{x}(X_b) = \sum_a x_a N_a(X_b) = x_b$（因为 $N_a(X_b) = \delta_{ab}$）。在单元内部，位移由顶点坐标线性插值——这就是"线性有限元"名字的来源。

#### 为什么这对 IPC 代码至关重要

同样的离散化也作用在测试函数 $Q(X)$ 上（Galerkin 方法）：

$$
Q(X) \approx \sum_a Q_a N_a(X)
$$

把这两个展开代入 Lec9 的 weak form（$\int \rho_0 Q \cdot a\,dX = \int Q \cdot T\,ds - \int \nabla Q : P\,dX$），就从"对任意连续函数 $Q$ 成立"变成了"对每个节点 $a$ 各写一个方程"——总共 $n \times 3$ 个方程（$n$ 个节点，每个 3 个分量）。

这意味着：**一旦你知道所有节点坐标 $x_a$，整个域内任何一点的位置都由 shape function 插值决定。** 所以 IPC 的全局状态向量 `IPCState::x` 就是所有节点坐标 $x_a$ 的堆叠：

$$
\mathbf{x} = [x_1^T, x_2^T, \ldots, x_n^T]^T \in \mathbb{R}^{3n}
$$

后续的所有操作——能量计算、梯度、Hessian——都只操作这个 $3n$ 维向量。连续 PDE 被彻底变成了有限维优化问题。

#### 关键点

- 线性四面体中，每个单元有 4 个节点，shape function 就是重心坐标（Lec 10 §3.2）
- 因为 $N_a$ 是线性函数，$\nabla N_a$ 是常量 → $F = \partial\hat{x}/\partial X$ 在单元内是常量（Lec 10 §5.2）——这极大简化了积分
- 1 个 3D 节点 = 3 个 DOF（x, y, z），$n$ 个节点 → $3n$ DOF

### 映射到代码

| 理论 | `IPCState` 实现 |
|------|----------------|
| 节点坐标 $x_a$ | `Eigen::VectorXd x`，大小 3N |
| 节点速度 $v_a$ | `Eigen::VectorXd v`，大小 3N |
| 对角质量 $m_a$ | `Eigen::VectorXd mass_diag`，大小 3N |
| 节点 $a$ 的位置 | `x.segment<3>(3*a)` |

---

### 复习内容：$D_m$、$D_m^{-1}$ 和 rest volume

**Lec 7 Section 1.4**（`lec7_strain_energy/lec7_course_note.md`）
**Lec 10 Section 5.1-5.2**（`lec10_finite_element/lec10_course_note.md`）

对 3D 四面体单元 $e = (v_0, v_1, v_2, v_3)$：

$$
D_m = [X_1 - X_0, \; X_2 - X_0, \; X_3 - X_0]
$$

$D_m$ 是参考构型中三条边向量组成的 3x3 矩阵。它完全由 rest positions 决定，**只计算一次**。

rest volume：

$$
V_e = \frac{|\det(D_m)|}{6}
$$

3D 四面体的体积公式。如果 $\det(D_m) \le 0$，说明 tet 退化或方向反了。

### 映射到代码

| 理论 | `TetBody` 实现 |
|------|---------------|
| $D_m$ | `precompute()` 中临时计算 |
| $D_m^{-1}$ | `std::vector<Eigen::Matrix3d> Dm_inv` — 预存 |
| $V_e$ | `std::vector<double> rest_volumes` — 预存 |
| 退化检查 | `rest_volumes[i] > 0`，否则抛异常 |

参考代码：`solid-sim-tutorial/6_inv_free/simulator.py` 中 `IB` 和 `vol` 的预计算。

---

### 复习内容：质量矩阵和 Mass Lumping

**Lec 10 Section 4**（`lec10_finite_element/lec10_course_note.md`）

Consistent mass matrix：

$$
M_{ab} = \int_{\Omega^0} \rho_0 N_a N_b \, dX
$$

Mass lumping 把它近似成对角矩阵：每个 tet 的总质量 $\rho V_e$ 均分给 4 个顶点。

$$
m_a = \sum_{e \ni a} \frac{\rho V_e}{4}
$$

Day 1 直接用 lumped mass（对角质量）。这是图形学仿真的标准做法（Lec 10 §4.3），代价是丢掉节点间惯性耦合，但换来对角质量矩阵——求逆直接取倒数，和 line search / contact 组合更方便。

### 映射到代码

```cpp
// TetBody::precompute() 中计算 per-vertex mass
std::vector<double> vertex_masses(vertex_count, 0.0);
for (size_t t = 0; t < tets.size(); ++t) {
    double tet_mass = density * rest_volumes[t];
    for (int k = 0; k < 4; ++k) {
        vertex_masses[tets[t][k]] += tet_mass / 4.0;
    }
}
// IPCSystem::initialize() 写入 mass_diag
for (size_t i = 0; i < vertex_count; ++i) {
    state.mass_diag[3*i+0] = vertex_masses[i];
    state.mass_diag[3*i+1] = vertex_masses[i];
    state.mass_diag[3*i+2] = vertex_masses[i];
}
```

---

## Phase 2 理论：能量模块

### 复习内容：优化重构（最核心的 insight）

**Lec 1 Section 4.3**（`lec1_discrete_space_and_time/lec1_course_note.md`）
**Lec 2 Section 3.1**（`lec2_mass_spring_systems/lec2_course_note.md`）

Backward Euler 等价于最小化增量势能：

$$
x^{n+1} = \arg\min_x \; E(x) = \underbrace{\frac{1}{2}\|x - \tilde{x}^n\|_M^2}_{E_I(x)} + \Delta t^2 \, P(x)
$$

其中预测位置 $\tilde{x}^n = x^n + \Delta t \, v^n$。

**物理直觉**（Lec 1 §4.3）：
- **惯性项** $E_I$：惩罚偏离"惯性预测位置"的程度——质量越大越难拉偏
- **势能项** $\Delta t^2 P$：弹性能 + 重力势能——希望位置处于低能构形
- 最小化 $E(x)$ = 在两者之间找平衡

**$\Delta t^2$ 的来源**（Lec 2 §3.1）：力通过 $h$ 变成速度增量，速度再通过 $h$ 变成位移增量，两次叠起来 = $h^2$。

### 映射到代码：惯性能量 `inertial_energy.hpp`

| 理论 | 代码 |
|------|------|
| $E_I = \frac{1}{2h^2}(x-\hat{x})^T M (x-\hat{x})$ | `compute_energy(x, x_hat, mass_diag, dt)` |
| $\nabla E_I = \frac{M}{h^2}(x-\hat{x})$ | `compute_gradient(...)` |
| $\nabla^2 E_I = \frac{M}{h^2}$（对角矩阵） | `compute_hessian_triplets(...)` |

注意：Lec 2 §3.2 指出惯性 Hessian 是质量矩阵（永远正定），不需要 PSD projection。

### $\hat{x}$ 的计算与重力处理

有两种等价的方式处理外力（Lec 1 §4.3）：

**方案 A：折入 $\hat{x}$**

$$
\hat{x} = x^n + h v^n + h^2 M^{-1} f_{ext}
$$

重力 $f_{ext} = mg$，$M^{-1}f_{ext} = g$（质量消掉），所以 $\hat{x}_i = x^n_i + hv^n_i + h^2g$。

**方案 B：独立 gravity energy（本项目采用）**

$$
\hat{x} = x^n + h v^n \quad \text{（纯惯性预测）}
$$

$$
E_{gravity}(x) = -f_g^T x = -\sum_a m_a \mathbf{g}^T \mathbf{x}_a
$$

两种方案数学等价：方案 B 展开梯度 $\nabla E_I + \nabla E_g = \frac{M}{h^2}(x - \hat{x}) - f_g$，令梯度为零解出的 $x$ 与方案 A 完全相同。

选择方案 B 的理由：
- $\hat{x}$ 保持纯运动学语义（无外力的惯性预测）
- 新增外力（barrier、friction）只需追加能量项，不需要修改 $\hat{x}$ 计算
- 每项能量可独立 log，方便数值调试

`GravityEnergy` 的特殊之处：它是关于 $x$ 的**线性**函数，所以梯度是常数，Hessian 为零。不贡献 Hessian triplets。

参考代码：`solid-sim-tutorial/6_inv_free/time_integrator.py` 中 `x_tilde = x + v * h`（该版本也把重力作为独立势能项处理）。

---

### 复习内容：Deformation Gradient $F$

**Lec 7 Section 1.2-1.4**（`lec7_strain_energy/lec7_course_note.md`）

$F$ 是 deformation map 对参考坐标的 Jacobian：

$$
F = \frac{\partial x}{\partial X}
$$

它描述的不是"点在哪"，而是"**点附近的一小块材料怎样被拉、压、剪、转**"。

离散 FEM 中：

$$
D_s = [x_1 - x_0, \; x_2 - x_0, \; x_3 - x_0], \quad F = D_s \cdot D_m^{-1}
$$

直觉验证（Lec 7 §1.4）：
- 没变形 → $D_s = D_m$ → $F = I$
- 纯旋转 → $D_s = R \cdot D_m$ → $F = R$
- 拉伸/压缩/剪切 → 编码在 $F$ 里

**$J = \det(F)$**（Lec 7 §2）：体积变化比。$J=1$ 不变，$J>1$ 膨胀，$0<J<1$ 压缩，$J \le 0$ 翻转。

### 映射到代码：`tet_elastic_assembler.hpp`

```cpp
// 对每个 tet t，取 4 个顶点当前位置
Eigen::Matrix3d Ds;
Ds.col(0) = x_v1 - x_v0;
Ds.col(1) = x_v2 - x_v0;
Ds.col(2) = x_v3 - x_v0;
Eigen::Matrix3d F = Ds * body.Dm_inv[t];
```

参考代码：`solid-sim-tutorial/6_inv_free/NeoHookeanEnergy.py` 中 `deformation_grad(x, elemVInd, IB)`。

---

### 复习内容：Strain Energy $\Psi(F)$ 和材料模型

**Lec 7 Section 4-5**（`lec7_strain_energy/lec7_course_note.md`）

好的 $\Psi(F)$ 应满足（Lec 7 §4.2）：

| 性质 | 含义 |
|------|------|
| Rigid null space | $\Psi(R) = 0$ — 纯旋转零能量 |
| Rotation invariance | $\Psi(RF) = \Psi(F)$ — 整体旋转不影响能量 |
| Isotropy | $\Psi(FR) = \Psi(F)$ — 材料方向不影响能量 |

**Day 1 选用 Fixed Corotated**（Lec 7 §5.2 的变体）：

$$
\Psi_{FC}(F) = \mu \|F - R\|_F^2 + \frac{\lambda}{2}(J - 1)^2
$$

其中 $R$ 是 $F$ 的 polar decomposition 提取的旋转部分：$F = RS$，$R = UV^T$。

选它而不是 Neo-Hookean 的原因：
- 实现更简单（不需要 $\ln J$，不需要 inversion-free filter）
- 数值更稳（$J \le 0$ 时不会直接炸掉）
- 先验证框架正确性，Day 2 再切换到 Neo-Hookean

**Lame 参数**（Lec 7 §5.5）：

$$
\mu = \frac{E}{2(1+\nu)}, \quad \lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}
$$

### 映射到代码：`tet_fixed_corotated_energy.hpp`

```cpp
// SVD: F = U * Sigma * V^T
// Polar decomposition: R = U * V^T (确保 det(R) = 1)
Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, Eigen::ComputeFullU | Eigen::ComputeFullV);
Eigen::Matrix3d U = svd.matrixU();
Eigen::Matrix3d V = svd.matrixV();
// 修正 reflection：如果 det(U)*det(V) < 0，翻转最小奇异值对应的列
if (U.determinant() * V.determinant() < 0) {
    U.col(2) *= -1;  // 翻转最后一列
}
Eigen::Matrix3d R = U * V.transpose();
double J = F.determinant();

// Energy
double energy = mu * (F - R).squaredNorm() + 0.5 * lambda * (J - 1) * (J - 1);
energy *= rest_volume;
```

---

### 复习内容：PK1 Stress 和从 $F$ 到 $x$ 的链式法则

**Lec 8 Section 2, 5**（`lec8_stress_and_derivatives/lec8_course_note.md`）
**Lec 10 Section 5.4-5.6**（`lec10_finite_element/lec10_course_note.md`）

First Piola-Kirchhoff stress：

$$
P = \frac{\partial \Psi}{\partial F}
$$

从 $P$ 到节点力（Lec 10 §5.4）：

$$
f^{int}_{e,a} = -V_e \cdot P_e \cdot \nabla N_a
$$

其中 $\nabla N_a$ 由 $D_m^{-1}$ 决定（Lec 10 §5.3）。对 3D 四面体，4 个顶点的形函数梯度是：

$$
\nabla N_1 = D_m^{-T} \begin{bmatrix}-1\\-1\\-1\end{bmatrix}, \quad
\nabla N_2 = D_m^{-T} \begin{bmatrix}1\\0\\0\end{bmatrix}, \quad
\nabla N_3 = D_m^{-T} \begin{bmatrix}0\\1\\0\end{bmatrix}, \quad
\nabla N_4 = D_m^{-T} \begin{bmatrix}0\\0\\1\end{bmatrix}
$$

且 $\sum_a \nabla N_a = 0$（partition of unity 推论，Lec 10 §5.3）。

### 映射到代码：gradient 装配

```cpp
// 对 tet t，计算 PK1 stress P (3x3)
Eigen::Matrix3d P = material.compute_pk1(F, ...);

// 形函数梯度：grad_N[1..3] 是 Dm_inv^T 的列
// grad_N[0] = -(grad_N[1] + grad_N[2] + grad_N[3])
Eigen::Matrix3d DmInvT = body.Dm_inv[t].transpose();
Eigen::Vector3d grad_N1 = DmInvT.col(0);  // 对应 v1
Eigen::Vector3d grad_N2 = DmInvT.col(1);  // 对应 v2
Eigen::Vector3d grad_N3 = DmInvT.col(2);  // 对应 v3
Eigen::Vector3d grad_N0 = -(grad_N1 + grad_N2 + grad_N3);  // 对应 v0

double V = body.rest_volumes[t];
// 节点力（弹性梯度 = -内力，即 +V*P*grad_N）
gradient.segment<3>(dof_v0) += V * P * grad_N0;
gradient.segment<3>(dof_v1) += V * P * grad_N1;
// ...
```

参考代码：`solid-sim-tutorial/6_inv_free/NeoHookeanEnergy.py` 中 `dPsi_div_dx(...)`。

---

### 复习内容：Hessian 和 diagonal space

**Lec 8 Section 3-4**（`lec8_stress_and_derivatives/lec8_course_note.md`）

Stress derivative $\partial P / \partial F$ 在 diagonal space 里分解为 A block 和 B blocks（Lec 8 §4.3）。

Hessian 从 $F$ 映射到 $x$（Lec 8 §5.3）：

$$
\frac{\partial^2 \Psi}{\partial x^2} = \left(\frac{\partial F}{\partial x}\right)^T \frac{\partial^2 \Psi}{\partial F^2} \left(\frac{\partial F}{\partial x}\right)
$$

线性四面体中 $F$ 对 $x$ 是线性的，所以没有额外的二阶项（Lec 8 §5.3）。

**PSD Projection**（Lec 8 §4.6）：如果某些 mode 给出负特征值，clamp 到零。因为 A 和 B 都是小矩阵，做特征分解成本极低。

### 映射到代码

Day 1 对于 Hessian 有两个选项：

**选项 A**：解析 Hessian（正确但推导复杂）
- 实现 diagonal space 的 A block 和 B blocks
- 做 PSD projection
- 通过 $U, V$ 旋回世界坐标
- 再通过 $dF/dx$ 映射到 12x12 局部矩阵

**选项 B**：有限差分 Hessian（Day 1 推荐的 fallback）
- 对 PK1 stress 做中心差分近似 $\partial P / \partial F$
- 精度 ~$O(\epsilon^2)$，够用于验证框架
- Day 2 再替换为解析版

```cpp
// FD Hessian 近似（每个 tet 的 12x12）
Eigen::Matrix<double, 12, 12> fd_hessian(
    const TetMaterialModel& material,
    const Eigen::Matrix3d& F,
    /* ... */
    double epsilon = 1e-6
) {
    // 对 12 个局部 DOF 分别扰动，差分 gradient
}
```

---

## Phase 3 理论：Newton Solver + Line Search

### 复习内容：Newton 法作为优化器

**Lec 1 Section 4.1, 4.3-4.4**（`lec1_discrete_space_and_time/lec1_course_note.md`）

Root-finding 和 optimization 是同一件事的两面（Lec 1 §4.3）：

$$
\nabla E(x) = 0 \quad \Leftrightarrow \quad \text{solve } g(x) = 0
$$

Newton 步 = 对能量做二阶近似后取极小值：

$$
E(x^i + p) \approx E(x^i) + \nabla E^T p + \frac{1}{2} p^T \nabla^2 E \cdot p
$$

对 $p$ 求导 = 0：

$$
\nabla^2 E(x^i) \cdot p = -\nabla E(x^i) \quad \Rightarrow \quad H \Delta x = -g
$$

Hessian 和 Jacobian 的关系（Lec 1 §4.1）：

$$
\nabla^2 E = M + \Delta t^2 \nabla^2 P = M - \Delta t^2 \frac{\partial f}{\partial x}
$$

### 映射到代码：`newton_solver.hpp`

```
每次迭代：
1. 装配 g = ∇E(x)           ← compute_total_gradient
2. 装配 H = ∇²E(x)          ← compute_total_hessian
3. Dirichlet 消元            ← 见下一节
4. 解 H Δx = -g             ← SimplicialLDLT
5. 检查 g^T Δx < 0          ← 是否下降方向
6. line search               ← 见下一节
7. x ← x + α Δx
8. 检查收敛
```

---

### 复习内容：Over-Shooting 和 Line Search

**Lec 1 Section 4.2, 4.4**（`lec1_discrete_space_and_time/lec1_course_note.md`）
**Lec 2 Section 4.3**（`lec2_mass_spring_systems/lec2_course_note.md`）

Over-shooting 问题（Lec 1 §4.2）：Newton 方向可能正确但步长过大。1D 例子 $E(x) = \sqrt{1+x^2}$：从 $x_0=2$ 出发，Newton 一步跳到 $x_1=-8$。

解决方案（Lec 1 §4.4）：

1. **SPD Projection**：保证 $H$ 正定 → 搜索方向一定是下降方向
2. **Backtracking Line Search**：先尝试 $\alpha=1$，如果能量没降就减半，直到 $E(x+\alpha p) < E(x)$

完整算法（Lec 1 §4.4）：

```
1. x^i ← x^n
2. repeat:
   - H_proj ← SPDProjection(∇²E(x^i))
   - p ← -H_proj^{-1} ∇E(x^i)
   - α ← BacktrackingLineSearch(x^i, p)
   - x^i ← x^i + α p
3. until ||p||_∞ / h < ε
4. x^{n+1} ← x^i, v^{n+1} ← (x^{n+1} - x^n) / Δt
```

每次迭代都保证 $E(x^{i+1}) < E(x^i)$ — 单调能量下降。

### 映射到代码：`line_search.hpp`

```cpp
// Armijo backtracking
// 条件：E(x + α p) ≤ E(x) + c · α · g^T p
// 其中 g^T p < 0（下降方向），c = 1e-4（Armijo 常数）
alpha = 1.0;
for (int i = 0; i < max_iter; ++i) {
    if (energy_fn(alpha) <= current_energy + armijo_c * alpha * directional_derivative)
        return {alpha, success};
    alpha *= 0.5;
}
```

参考代码：`solid-sim-tutorial/6_inv_free/time_integrator.py` 中的 line search 实现。

---

### 复习内容：Hessian PSD Projection

**Lec 1 Section 4.4**（`lec1_discrete_space_and_time/lec1_course_note.md`）
**Lec 8 Section 4.6**（`lec8_stress_and_derivatives/lec8_course_note.md`）

全局 Hessian 不一定正定。SPD projection 做法：

$$
H_{proj} = Q \cdot \text{diag}(\max(\lambda_1, \delta), \ldots, \max(\lambda_n, \delta)) \cdot Q^T
$$

实际中在**单元级别**做 PSD projection（Lec 8 §4.6）更高效：对每个 tet 的 12x12 局部 Hessian 做 eigendecomposition + clamp，再装配到全局。因为局部矩阵小，成本极低。

Day 1 简化版：先用全局对角 regularization $H + \epsilon I$（$\epsilon = 10^{-8}$）。Day 2 再加 per-element PSD projection。

---

## Phase 4 理论：Dirichlet 边界条件

### 复习内容

**Lec 3 全文**（`lec3_dirichlet_boundary_conditions/lec3_course_note.md`）

核心问题：固定某些顶点不动，怎么并入 Newton 迭代？

**KKT 系统**（Lec 3 §3.3）：最完整但最贵的做法。

**DOF Elimination**（Lec 3 §4）：Day 1 用这个。

思路（Lec 3 §4.2-4.3）：
- 当前点已满足约束 $Ax = b$
- Newton 增量必须满足 $A\Delta x = 0$
- 对 sticky DBC，$A$ 是 selection matrix，所以 $\Delta x_B = 0$
- 不需要重排 DOF，直接在原系统上操作：
  1. 受约束行列的非对角元清零
  2. 对角元设为 1
  3. 对应梯度分量设为 0

**数值例子**（Lec 3 §4.3）：

$$
\tilde{H} = \begin{bmatrix}
4 & -1 & 0 & 0\\
-1 & 4 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}, \quad
\tilde{g} = \begin{bmatrix}
1\\ 2\\ 0\\ 0
\end{bmatrix}
$$

解出来固定 DOF 的 $\Delta x = 0$，自动满足约束。

**Line search 不会破坏约束**（Lec 3 §4.4）：因为 $\Delta x_B = 0$，不管 $\alpha$ 是多少，$x_B + \alpha \cdot 0 = x_B$。

### 映射到代码

在 `newton_solver.hpp` 中，解线性系统前：

```cpp
// 对所有 fixed DOFs：
for (size_t i = 0; i < dof_count; ++i) {
    if (!free_dof_mask[i]) {
        gradient[i] = 0.0;
        // Hessian: 在 triplet 阶段就不写 fixed DOF 的 off-diagonal
        // 对角写 1.0
    }
}
```

参考代码：`solid-sim-tutorial/2_dirichlet/time_integrator.py` 中 `search_dir(...)` 的实现完全对应 Lec 3 §5.3 的代码。

---

## Phase 5 理论：总能量装配与验证

### 复习内容：单元→全局装配

**Lec 2 Section 3.5-3.7**（`lec2_mass_spring_systems/lec2_course_note.md`）

局部 Hessian 装配到全局稀疏矩阵的过程：

1. 每个 tet 产生 12x12 局部 Hessian（4 顶点 × 3 DOF）
2. 用全局 DOF 索引 scatter 到全局 triplets
3. 用 `Eigen::SparseMatrix::setFromTriplets` 构建

```cpp
// 对 tet t 的 4 个顶点 v[0..3]
for (int a = 0; a < 4; ++a) {
    for (int b = 0; b < 4; ++b) {
        for (int di = 0; di < 3; ++di) {
            for (int dj = 0; dj < 3; ++dj) {
                int row = 3 * v[a] + di;
                int col = 3 * v[b] + dj;
                triplets.emplace_back(row, col, local_H(3*a+di, 3*b+dj));
            }
        }
    }
}
```

### 复习内容：有限差分验证

这不在课程笔记里，但是 IPC 从零实现的最重要工程保障。

对任意能量函数 $E(x)$，验证 gradient：

$$
\frac{\partial E}{\partial x_i} \approx \frac{E(x + \epsilon e_i) - E(x - \epsilon e_i)}{2\epsilon}
$$

验证 Hessian（用 gradient 的差分）：

$$
H_{ij} \approx \frac{g_i(x + \epsilon e_j) - g_i(x - \epsilon e_j)}{2\epsilon}
$$

Day 1 的 FD 测试优先级：
1. inertial energy gradient（二次型，应精确到 $10^{-10}$）
2. tet elastic energy gradient（非线性，精度 $\sim 10^{-5}$）

---

## 理论→代码总结表

| 课程笔记 | Section | Day 1 代码文件 | 对应内容 |
|----------|---------|---------------|----------|
| Lec 1 | §4.3 | `inertial_energy.hpp` | $E_I$ 的定义和导数 |
| Lec 1 | §4.4 | `line_search.hpp` | Backtracking line search |
| Lec 1 | §4.4 | `newton_solver.hpp` | Projected Newton 主循环 |
| Lec 2 | §3.5-3.7 | `tet_elastic_assembler.hpp` | 局部→全局装配 |
| Lec 3 | §4 | `newton_solver.hpp` | Dirichlet DOF elimination |
| Lec 7 | §1.4 | `tet_elastic_assembler.hpp` | $F = D_s D_m^{-1}$ |
| Lec 7 | §3 | `tet_fixed_corotated_energy.hpp` | SVD / polar decomposition |
| Lec 7 | §5.2 | `tet_fixed_corotated_energy.hpp` | Fixed corotated $\Psi$ |
| Lec 7 | §5.5 | `tet_material_model.hpp` | Lame 参数 |
| Lec 8 | §2 | `tet_fixed_corotated_energy.hpp` | PK1 stress $P$ |
| Lec 8 | §4-5 | `tet_fixed_corotated_energy.hpp` | Stress derivative / Hessian |
| Lec 10 | §3 | `tet_body.hpp` | Shape functions = 重心坐标 |
| Lec 10 | §4 | `tet_body.hpp` + `ipc_system.hpp` | Mass lumping |
| Lec 10 | §5 | `tet_elastic_assembler.hpp` | $f^{int} = -V_e P \nabla N_a$ |

---

## 推荐复习顺序

如果你只有 1.5 小时复习时间：

1. **Lec 1 §4.3**（15 min）— 优化重构，理解 $E(x)$ 两项的物理意义
2. **Lec 7 §1.4**（10 min）— $F = D_s D_m^{-1}$ 的推导和直觉
3. **Lec 7 §5.2**（10 min）— Fixed corotated 或 §5.5 Neo-Hookean 公式
4. **Lec 8 §2**（10 min）— PK1 stress 定义
5. **Lec 10 §5.1-5.5**（15 min）— $F$ 在单元内是常量，节点力 $= -V_e P \nabla N_a$
6. **Lec 3 §4**（15 min）— DOF elimination 实现方式
7. **Lec 1 §4.4**（10 min）— Backtracking line search
8. **Lec 2 §3.5-3.7**（10 min）— 局部→全局稀疏装配

如果你对 Lec 1 和 Lec 7 已经熟悉，可以跳过 1-3，把时间集中在 Lec 10 §5 和 Lec 3 §4。
