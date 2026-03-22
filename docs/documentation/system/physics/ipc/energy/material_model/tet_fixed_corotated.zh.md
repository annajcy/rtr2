# `tet_fixed_corotated.hpp`

`src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp` 实现了当前具体使用的 tet 材料：`FixedCorotatedMaterial`。

## 接口

`FixedCorotatedMaterial` 提供三个 `const` 方法，满足 `TetMaterialModel` concept：

- `compute_energy(F, rest_volume, youngs_modulus, poisson_ratio)` → `double`
- `compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio)` → `Matrix3d`
- `compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio)` → `Matrix<double, 9, 9>`

## Lame 参数

工程参数（Young's modulus $E$, Poisson ratio $\nu$）→ Lame 参数：

$$
\mu = \frac{E}{2(1+\nu)}, \qquad \lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}
$$

- $\mu$（剪切模量）：抵抗形状变化
- $\lambda$：抵抗体积变化（$\nu \to 0.5$ 时 $\lambda \to \infty$，趋近不可压）

**代码**：`compute_lame_parameters(youngs_modulus, poisson_ratio)` → `LameParameters{mu, lambda}`

## SVD 与极分解

Fixed corotated 的核心工具是 **极分解** $F = RS$，通过 SVD 实现：

$$
F = U \Sigma V^T, \qquad R = U D V^T
$$

SVD 不保证 $\det(UV^T) = +1$，需要符号修正：

```
如果 det(U * V^T) < 0:
    D = diag(1, 1, -1)      // 翻转最后一个奇异值
    signed_singular_values[2] *= -1
否则:
    D = I
```

修正后 $R = UDV^T$ 是真正的旋转矩阵。带符号奇异值 $\hat{\sigma}$ 让极分解在 $J < 0$（翻转 tet）时也有定义。

**代码**：`compute_rotation_data(F)` → `RotationData{UD, V, R, signed_singular_values}`

保存 `UD` 和 `V` 是因为 Hessian 计算的 $dR$ 推导需要它们。

## Energy: $\Psi(F)$

Fixed corotated 能量密度：

$$
\Psi(F) = \mu \|F - R\|_F^2 + \frac{\lambda}{2}(J - 1)^2
$$

- 第一项：惩罚变形偏离纯旋转的程度。$F = R$ 时为 0（rigid null space）
- 第二项：惩罚体积变化。$J = \det(F) = 1$ 时为 0

总能量 = 能量密度 × rest volume：$E = V_e \cdot \Psi(F)$

用 SVD 展开第一项可以证明它只依赖奇异值：

$$
\|F - R\|_F^2 = \|U\hat{\Sigma}V^T - UDV^T\|_F^2 = \|\hat{\Sigma} - D\|_F^2 = \sum_i (\hat{\sigma}_i - d_i)^2
$$

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

## PK1 Stress: $P = \partial\Psi / \partial F$

分两项求导。

### 第一项：$\partial / \partial F \|F - R\|_F^2 = 2(F - R)$

虽然 $R$ 依赖 $F$，但 $R(F)$ 是 $F$ 在旋转群 $SO(3)$ 上的最近点投影——投影点的变化方向与 $(F-R)$ 正交，所以可以当 $R$ 是常数求导。

严格证明：$\|F-R\|_F^2 = \|F\|_F^2 - 2\text{tr}(R^TF) + 3$。$\text{tr}(R^TF) = \sum \hat{\sigma}_i$，其关于 $F$ 的导数恰好是 $R$。所以 $\partial/\partial F = 2F - 2R$。

### 第二项：$\partial/\partial F \frac{\lambda}{2}(J-1)^2 = \lambda(J-1) \cdot \text{cof}(F)$

$J = \det(F)$ 对 $F$ 的导数是 **cofactor matrix**：

$$
\frac{\partial J}{\partial F} = \text{cof}(F)
$$

等价于 $J F^{-T}$（$F$ 可逆时），但 cofactor 形式在 $F$ 奇异时也有定义。

cofactor 的三列是 $F$ 另外两列的叉积：

$$
\text{cof}(F) = [F_1 \times F_2, \; F_2 \times F_0, \; F_0 \times F_1]
$$

**代码**：`compute_cofactor_matrix(F)` 直接用叉积。

### 合并

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

注意返回的是 $\partial\Psi/\partial F$（不含 $V_e$），$V_e$ 在装配层 `MaterialEnergy` 中乘入。

**验证**：
- $F = I$：$R = I$，$J = 1$ → $P = 0$ ✓
- $F = 2I$：$P = 2\mu I + 28\lambda I$，体积膨胀 → 正应力 ✓

## Hessian: $\partial P / \partial F$（9×9 矩阵）

Hessian 是 PK1 对 $F$ 的导数，把 3×3 的 $F$ 和 $P$ 展平为 9 维向量后是 9×9 矩阵。

对 $P$ 取微分（给 $F$ 扰动 $dF$）：

$$
dP = 2\mu(dF - dR) + \lambda\big(dJ \cdot \text{cof}(F) + (J-1) \cdot d\text{cof}(F)\big)
$$

三个微分量：

### $dJ$：行列式的微分

$$
dJ = \text{cof}(F) : dF = \sum_{ij} \text{cof}(F)_{ij} \, dF_{ij}
$$

**代码**：`cofactor.cwiseProduct(dF).sum()`

### $d\text{cof}(F)$：cofactor 的微分

对 $\text{cof}(F) = [F_1 \times F_2, \; F_2 \times F_0, \; F_0 \times F_1]$ 用乘法法则：

$$
d\text{cof}(F) = \begin{bmatrix}
dF_1 \times F_2 + F_1 \times dF_2, &
dF_2 \times F_0 + F_2 \times dF_0, &
dF_0 \times F_1 + F_0 \times dF_1
\end{bmatrix}
$$

**代码**：`differential_cofactor_matrix(F, dF)`

### $dR$：旋转的微分（最复杂）

$R$ 是正交矩阵，$dR$ 一定是 $R$ 乘以反对称矩阵：$dR = R \cdot \hat{\omega}$

（因为 $R^T dR + dR^T R = 0$ → $R^T dR$ 反对称。）

**求 $\hat{\omega}$**：

定义 $M = R^T dF - dF^T R$（反对称），$B = V^T M V$。

在 $V$ 基下，$\hat{\omega}$ 的表示 $\tilde{\omega} = V^T \hat{\omega} V$ 的上三角元素为：

$$
\tilde{\omega}_{ij} = \frac{B_{ij}}{\hat{\sigma}_i + \hat{\sigma}_j}, \qquad i < j
$$

最终：$dR = UD \cdot \tilde{\omega} \cdot V^T$

**推导过程**：

从 $R^T F = D\hat{\Sigma}$（对角）出发，两边求微分：

$$
dR^T F + R^T dF = d(D\hat{\Sigma})
$$

代入 $dR = R\hat{\omega}$，得 $-\hat{\omega} D\hat{\Sigma} + R^T dF = d(D\hat{\Sigma})$。

取反对称部分（对角部分对应 $d\hat{\Sigma}$），在 $V$ 基下提取出 $\tilde{\omega}_{ij} = B_{ij} / (\hat{\sigma}_i + \hat{\sigma}_j)$。

**代码**：

```cpp
inline Matrix3d differential_rotation(const Matrix3d& dF, const RotationData& rot) {
    Matrix3d M = rot.R.transpose() * dF - dF.transpose() * rot.R;
    Matrix3d B = rot.V.transpose() * M * rot.V;
    Matrix3d omega_hat = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            double denom = rot.signed_singular_values[i] + rot.signed_singular_values[j];
            if (std::abs(denom) <= 1e-12) continue;  // 退化保护
            double omega_ij = B(i, j) / denom;
            omega_hat(i, j) = omega_ij;
            omega_hat(j, i) = -omega_ij;
        }
    return rot.UD * omega_hat * rot.V.transpose();
}
```

**退化保护**：$\hat{\sigma}_i + \hat{\sigma}_j \approx 0$ 时跳过该模态（几乎完全压扁的 tet，贡献极小）。

### 逐列构建 9×9

对 $j = 0, \ldots, 8$，令 $dF = e_j$（标准基 reshape 成 3×3），算 $dP$，展平为 Hessian 第 $j$ 列：

```cpp
Matrix<double, 9, 9> compute_hessian(...) const {
    auto lame = compute_lame_parameters(youngs_modulus, poisson_ratio);
    auto rotation_data = compute_rotation_data(F);
    double J = F.determinant();
    Matrix3d cofactor = compute_cofactor_matrix(F);

    Matrix<double, 9, 9> hessian = Matrix<double, 9, 9>::Zero();
    for (int j = 0; j < 9; ++j) {
        Matrix3d dF = Matrix3d::Zero();
        Map<Matrix<double, 9, 1>>(dF.data())[j] = 1.0;

        Matrix3d dR = differential_rotation(dF, rotation_data);
        Matrix3d dCof = differential_cofactor_matrix(F, dF);
        double dJ = cofactor.cwiseProduct(dF).sum();

        Matrix3d dP = 2.0 * lame.mu * (dF - dR)
                     + lame.lambda * (dJ * cofactor + (J - 1.0) * dCof);

        hessian.col(j) = flatten_matrix(dP);
    }
    return hessian;
}
```

逐列构建而非闭式的理由：代码直接对应 $dP$ 定义，容易验证；复用 helper 函数；方便 FD 逐列验证；9 次 3×3 运算开销可接受。
