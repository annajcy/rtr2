# `material_energy.hpp`

`src/rtr/system/physics/ipc/energy/material_energy.hpp` 把单个 tet 上定义的 `TetMaterialModel` 桥接回全局 IPC energy 接口。

## 它解决什么问题

材料模型在 $F$ 空间工作（3×3 矩阵），solver 在全局 DOF 空间工作（$3n$ 维向量）。`MaterialEnergy<Material>` 完成两者之间的转换：

- 材料给出：$\Psi(F)$, $P = \partial\Psi/\partial F$ (3×3), $\partial^2\Psi/\partial F^2$ (9×9)
- Solver 需要：总能量标量, 全局 DOF 梯度 (3n), 全局 Hessian triplets

转换涉及三层链式法则：

$$
E(x) = \sum_e V_e \cdot \Psi(F_e), \quad F_e = D_s(x) \cdot D_m^{-1}
$$

## 前置工具函数

### `build_Ds`：从全局 DOF 提取当前构型边向量

给定 tet $(v_0, v_1, v_2, v_3)$ 和全局位置向量 $x$：

$$
D_s = [x_{v_1} - x_{v_0}, \quad x_{v_2} - x_{v_0}, \quad x_{v_3} - x_{v_0}]
$$

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

`dof_offset` 是 body 在全局 DOF 向量中的起始位置（多 body 时，第二个 body 从 `3 * vertex_count_of_body_0` 开始）。

### `compute_shape_gradients`：计算 $\nabla N_a$

shape function 梯度由 $D_m^{-1}$ 决定：

$$
\nabla N_1 = D_m^{-T} e_0, \quad \nabla N_2 = D_m^{-T} e_1, \quad \nabla N_3 = D_m^{-T} e_2
$$

$$
\nabla N_0 = -(\nabla N_1 + \nabla N_2 + \nabla N_3) \qquad \text{(partition of unity)}
$$

```cpp
inline array<Vector3d, 4> compute_shape_gradients(const Matrix3d& Dm_inv) {
    Matrix3d G = Dm_inv.transpose();
    array<Vector3d, 4> gradients;
    gradients[1] = G.col(0);
    gradients[2] = G.col(1);
    gradients[3] = G.col(2);
    gradients[0] = -(gradients[1] + gradients[2] + gradients[3]);
    return gradients;
}
```

### `build_dFdx_matrix`：$\partial \text{vec}(F) / \partial x_a$（9×3 矩阵）

这个矩阵编码了"移动节点 $a$ 如何改变 $F$"。推导从 $F = D_s \cdot D_m^{-1}$ 出发。

#### 推导 $\delta F = \delta x_a \otimes \nabla N_a$

回顾 $D_s = [x_1 - x_0, \; x_2 - x_0, \; x_3 - x_0]$，所以 $F = D_s \cdot D_m^{-1}$。

**当 $a \in \{1, 2, 3\}$ 时**：给 $x_a$ 一个微小扰动 $\delta x_a$（3D 向量），只有 $D_s$ 的第 $(a{-}1)$ 列改变：

$$
\delta D_s = \delta x_a \cdot e_{a-1}^T
$$

其中 $e_{a-1}$ 是标准基向量（$e_0 = [1,0,0]^T$ 等）。$\delta x_a \cdot e_{a-1}^T$ 是外积，得到一个秩为 1 的 3×3 矩阵：只有第 $(a{-}1)$ 列非零，值为 $\delta x_a$。

代入 $F = D_s \cdot D_m^{-1}$：

$$
\delta F = \delta D_s \cdot D_m^{-1} = (\delta x_a \cdot e_{a-1}^T) \cdot D_m^{-1} = \delta x_a \cdot \underbrace{(e_{a-1}^T D_m^{-1})}_{\text{1×3 行向量}}
$$

$e_{a-1}^T D_m^{-1}$ 是 $D_m^{-1}$ 的第 $(a{-}1)$ 行，转置后等于 $D_m^{-T}$ 的第 $(a{-}1)$ 列。回顾前面定义的 $\nabla N_a = D_m^{-T} e_{a-1}$，所以：

$$
\boxed{\delta F = \delta x_a \otimes \nabla N_a}
$$

这是一个 3×3 矩阵的外积：$(\delta F)_{ij} = (\delta x_a)_i \cdot (\nabla N_a)_j$。

**当 $a = 0$ 时**：$x_0$ 出现在 $D_s$ 的三列中（被减数），所以移动 $x_0$ 同时改变三列：

$$
\delta D_s = [-\delta x_0, \; -\delta x_0, \; -\delta x_0]
$$

$$
\delta F = \delta D_s \cdot D_m^{-1} = -\delta x_0 \cdot [1,1,1] \cdot D_m^{-1} = \delta x_0 \otimes \underbrace{(-D_m^{-T}(e_0 + e_1 + e_2))}_{\nabla N_0}
$$

这正好等于 $\delta x_0 \otimes \nabla N_0$，因为 $\nabla N_0 = -(\nabla N_1 + \nabla N_2 + \nabla N_3)$。

所以公式对所有 4 个顶点统一成立：$\delta F = \delta x_a \otimes \nabla N_a$。

**数值验证**：取参考 tet $D_m = I$，$\nabla N_1 = (1,0,0)$。给 $x_1$ 一个扰动 $\delta x_1 = (0, \epsilon, 0)$：

$$
\delta F = \begin{bmatrix}0\\\epsilon\\0\end{bmatrix} \otimes \begin{bmatrix}1\\0\\0\end{bmatrix} = \begin{bmatrix}0&0&0\\\epsilon&0&0\\0&0&0\end{bmatrix}
$$

只有 $\delta F_{10} = \epsilon$ 非零——把 $x_1$（x 方向的顶点）往 y 方向推，导致 $F$ 的 $(1,0)$ 分量变化。直接用 $\delta D_s$ 验算也得到相同结果。✓

#### 展平为 9×3 矩阵

有了 $\delta F = \delta x_a \otimes \nabla N_a$，现在要把它写成矩阵乘法的形式。展平后：

$$
\text{vec}(\delta F) = \underbrace{\frac{\partial \text{vec}(F)}{\partial x_a}}_{9 \times 3} \cdot \delta x_a
$$

$(\delta F)_{ij} = (\delta x_a)_i \cdot (\nabla N_a)_j$，在 Eigen 列优先展平（$F_{ij} \to \text{index } i + 3j$）下：

$$
\text{dFdx}_{(i+3j),\, i} = (\nabla N_a)_j
$$

用数值例子：$\nabla N_a = (g_0, g_1, g_2)$ 时：

$$
\text{dFdx}_a = \begin{bmatrix}
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

每 3 行一组对应 $F$ 的一列，每组内是 $(\nabla N_a)_j \cdot I_3$。

```cpp
inline Matrix<double, 9, 3> build_dFdx_matrix(const Vector3d& grad_N) {
    Matrix<double, 9, 3> dFdx = Matrix<double, 9, 3>::Zero();
    for (int col = 0; col < 3; ++col)        // F 的列 j
        for (int row = 0; row < 3; ++row)     // F 的行 i = δx_a 的分量
            dFdx(row + 3 * col, row) = grad_N[col];
    return dFdx;
}
```

## Energy 装配

$$
E_{elastic}(x) = \sum_e V_e \cdot \Psi(F_e)
$$

遍历所有 tet，逐个算 $F$，调材料的 `compute_energy`，累加。

```cpp
static double compute_energy(const Input& input) {
    double total = 0.0;
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        Matrix3d Ds = build_Ds(input.x, input.body.info.dof_offset, tet);
        Matrix3d F = Ds * input.body.geometry.Dm_inv[t];
        total += input.material.compute_energy(F, rest_volumes[t], E, nu);
    }
    return total;
}
```

注意 `compute_energy` 返回的已经是 $V_e \cdot \Psi(F)$（rest_volume 在材料内部乘入）。

## Gradient 装配

### 理论

回顾节点力公式：

$$
\frac{\partial E_e}{\partial x_a} = V_e \cdot P_e \cdot \nabla N_a
$$

这是一个 3D 向量。多个 tet 共享顶点时，梯度自然累加（`+=` 模式）。

### 代码

```cpp
static void compute_gradient(const Input& input, VectorXd& gradient) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        Matrix3d Ds = build_Ds(...);
        Matrix3d F = Ds * Dm_inv[t];

        // 材料给出 P = ∂Ψ/∂F (3×3)
        Matrix3d P = input.material.compute_pk1(F, V, E, nu);

        // ∇N_a：4 个 shape function 梯度
        auto grad_N = compute_shape_gradients(Dm_inv[t]);

        // 对 4 个顶点分别累加
        for (int a = 0; a < 4; ++a) {
            Vector3d local_grad = V * P * grad_N[a];
            gradient.segment<3>(dof_offset + 3 * tet[a]) += local_grad;
        }
    }
}
```

`P * grad_N[a]` 是矩阵乘向量（3×3 × 3×1 = 3×1）。$P$ 编码了"每个参考方向的应力"，$\nabla N_a$ 编码了"节点 $a$ 影响的参考方向"。两者相乘 = "节点 $a$ 从应力场中分到的力"。

### 数值验证

参考 tet（$D_m = I$），$F = \text{diag}(2,1,1)$，$\mu=1, \lambda=1, V_e=1/6$：

$P = \text{diag}(2, 0, -1)$，$\nabla N_1 = (1, 0, 0)$

$$
\text{grad}_{v_1} = \tfrac{1}{6} \cdot \text{diag}(2,0,-1) \cdot (1,0,0)^T = \tfrac{1}{6}(2, 0, 0)^T
$$

$v_1$ 沿 x 正方向受梯度（继续拉伸会增能）。✓

$\nabla N_0 = (-1,-1,-1)$：

$$
\text{grad}_{v_0} = \tfrac{1}{6} \cdot \text{diag}(2,0,-1) \cdot (-1,-1,-1)^T = \tfrac{1}{6}(-2, 0, 1)^T
$$

## Hessian 装配

### 理论推导

**目标**：计算 tet $e$ 对全局 Hessian 中 $(v_a, v_b)$ block 的贡献（3×3 矩阵）。

从 gradient 再求一次导：

$$
\frac{\partial^2 E_e}{\partial x_a \partial x_b} = V_e \cdot \frac{\partial (P \cdot \nabla N_a)}{\partial x_b}
$$

$\nabla N_a$ 是常数，所以 = $V_e \cdot \frac{\partial P}{\partial x_b} \cdot \nabla N_a$。

直接操作 $\partial P / \partial x_b$（4 阶张量）很痛苦。**关键技巧**：用展平 + 链式法则绕开。

把 gradient 写成展平形式：

$$
\text{vec}\left(\frac{\partial E_e}{\partial x_a}\right) = V_e \cdot \text{dFdx}_a^T \cdot \text{vec}(P)
$$

再对 $x_b$ 求导：

$$
\boxed{H_{ab}^{(e)} = V_e \cdot \text{dFdx}_a^T \cdot H_F \cdot \text{dFdx}_b}
$$

其中 $H_F = \partial^2\Psi/\partial F^2$ 是材料返回的 9×9 Hessian，$\text{dFdx}_a$ 是 9×3 的 Jacobian。

**维度验证**：$(3 \times 9) \cdot (9 \times 9) \cdot (9 \times 3) = 3 \times 3$ ✓

每个 tet 有 4 个顶点 → $4 \times 4 = 16$ 个 3×3 block → $144$ 个 scalar triplet。

### 对称化

材料的 `compute_hessian` 可能因数值精度返回不完全对称的 9×9 矩阵。装配前做显式对称化：

```cpp
hessian_F = 0.5 * (hessian_F + hessian_F.transpose());
```

保证全局 Hessian 对称（`SimplicialLDLT` 要求）。

### 代码

```cpp
static void compute_hessian_triplets(const Input& input,
                                     vector<Triplet<double>>& triplets) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        Matrix3d F = Ds * Dm_inv[t];
        double V = rest_volumes[t];

        // 材料给出 9×9 的 ∂²Ψ/∂F²
        Matrix<double, 9, 9> H_F = input.material.compute_hessian(F, V, E, nu);
        H_F = 0.5 * (H_F + H_F.transpose());  // 显式对称化

        // 预计算 4 个 dFdx 矩阵 (9×3)
        auto grad_N = compute_shape_gradients(Dm_inv[t]);
        array<Matrix<double, 9, 3>, 4> dFdx;
        for (int a = 0; a < 4; ++a)
            dFdx[a] = build_dFdx_matrix(grad_N[a]);

        // 装配 4×4 = 16 个 3×3 block
        for (int a = 0; a < 4; ++a) {
            Index row_base = dof_offset + 3 * tet[a];
            for (int b = 0; b < 4; ++b) {
                Index col_base = dof_offset + 3 * tet[b];

                // 核心公式
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

### 数据流

```
对每个 tet e:
    x (3n 全局 DOF)
    │
    ├── build_Ds() ──→ D_s (3×3)
    │                   │
    │                   └── F = D_s * Dm_inv  ──→ F (3×3)
    │                                              │
    │                          ┌────────────────────┤
    │                          │                    │
    │                          ▼                    ▼
    │                 material.compute_pk1   material.compute_hessian
    │                          │                    │
    │                          ▼                    ▼
    │                        P (3×3)             H_F (9×9)
    │                          │                    │
    ├── compute_shape_gradients() ──→ ∇N_a (4 个 3D 向量)
    │                          │            │
    │                          │    build_dFdx_matrix()
    │                          │            │
    │                          │            ▼
    │                          │    dFdx_a (4 个 9×3)
    │                          │            │
    │                          ▼            ▼
    │               gradient += V*P*∇N_a   H_ab = V * dFdx_a^T * H_F * dFdx_b
    │                                       │
    │                                       ▼
    │                              triplets.emplace_back(...)
    │                              (16 blocks × 9 entries = 144 triplets per tet)
```

### 数值例子

参考 tet（$D_m = I$），$F = I$（无变形），$\mu = 1, \lambda = 1, V_e = 1/6$。

$\nabla N_1 = (1,0,0)$ → $\text{dFdx}_1$ 只在 $(0,0), (1,1), (2,2)$ 处为 1，即只选 $H_F$ 的左上 3×3 block。

$\nabla N_2 = (0,1,0)$ → $\text{dFdx}_2$ 只在 $(3,0), (4,1), (5,2)$ 处为 1，选 $H_F$ 的中间 block。

**Block $H_{11}$**（顶点 1 对自己）：

$$
H_{11} = \tfrac{1}{6} \cdot \text{dFdx}_1^T \cdot H_F \cdot \text{dFdx}_1
$$

在 $F = I$ 时 $H_F$ 左上 3×3 $\approx \text{diag}(2\mu + \lambda, 2\mu, 2\mu) = \text{diag}(3, 2, 2)$。

$$
H_{11} \approx \tfrac{1}{6}\text{diag}(3, 2, 2)
$$

物理含义：沿 x 拉伸 $v_1$ 的刚度最大（拉伸模式 = $\mu + \lambda$），y/z 方向是纯剪切（$= \mu$）。✓

## 计算复杂度

| 操作 | 复杂度 |
|------|--------|
| `build_Ds` + `F = Ds * Dm_inv` | $O(27)$ |
| `compute_pk1`（含 SVD） | $O(100+)$ |
| `compute_hessian`（9 列 × SVD 微分） | $O(900+)$ |
| `dFdx_a^T * H_F * dFdx_b`（16 blocks） | $O(16 \times 81)$ |
| Triplet 写入（144 个） | $O(144)$ |

gradient 装配比 Hessian 便宜一个数量级（不需 9×9 Hessian 和 16 个矩阵乘法）。
