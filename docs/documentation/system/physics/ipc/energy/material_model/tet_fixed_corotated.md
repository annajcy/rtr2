# `tet_fixed_corotated.hpp`

`src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp` implements the concrete tet material: `FixedCorotatedMaterial`.

## API

`FixedCorotatedMaterial` provides three `const` methods satisfying the `TetMaterialModel` concept:

- `compute_energy(F, rest_volume, youngs_modulus, poisson_ratio)` → `double`
- `compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio)` → `Matrix3d`
- `compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio)` → `Matrix<double, 9, 9>`

## Lame Parameters

Engineering parameters (Young's modulus $E$, Poisson ratio $\nu$) → Lame parameters:

$$
\mu = \frac{E}{2(1+\nu)}, \qquad \lambda = \frac{E\nu}{(1+\nu)(1-2\nu)}
$$

- $\mu$ (shear modulus): resists shape change
- $\lambda$: resists volume change ($\nu \to 0.5$ → $\lambda \to \infty$, nearly incompressible)

**Code**: `compute_lame_parameters(youngs_modulus, poisson_ratio)` → `LameParameters{mu, lambda}`

## SVD and Polar Decomposition

The core tool is **polar decomposition** $F = RS$, computed via SVD:

$$
F = U \Sigma V^T, \qquad R = U D V^T
$$

SVD does not guarantee $\det(UV^T) = +1$, so a sign correction is needed:

```
if det(U * V^T) < 0:
    D = diag(1, 1, -1)       // flip the last singular value
    signed_singular_values[2] *= -1
else:
    D = I
```

After correction, $R = UDV^T$ is a proper rotation. The signed singular values $\hat{\sigma}$ make the decomposition well-defined even when $J < 0$ (inverted tet).

**Code**: `compute_rotation_data(F)` → `RotationData{UD, V, R, signed_singular_values}`

`UD` and `V` are stored because the Hessian's $dR$ computation needs them.

## Energy: $\Psi(F)$

Fixed corotated energy density:

$$
\Psi(F) = \mu \|F - R\|_F^2 + \frac{\lambda}{2}(J - 1)^2
$$

- First term: penalizes deviation from pure rotation. Zero when $F = R$ (rigid null space).
- Second term: penalizes volume change. Zero when $J = \det(F) = 1$.

Total energy = density × rest volume: $E = V_e \cdot \Psi(F)$.

The first term depends only on the singular values (Frobenius norm is rotation-invariant):

$$
\|F - R\|_F^2 = \|\hat{\Sigma} - D\|_F^2 = \sum_i (\hat{\sigma}_i - d_i)^2
$$

**Code**:

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

Differentiate each term separately.

### First term: $\partial / \partial F \|F - R\|_F^2 = 2(F - R)$

Although $R$ depends on $F$, $R(F)$ is the nearest-point projection of $F$ onto $SO(3)$. The projection moves tangent to the constraint surface, orthogonal to $(F - R)$, so it does not contribute.

Formal proof: $\|F-R\|_F^2 = \|F\|_F^2 - 2\text{tr}(R^TF) + 3$. Since $\text{tr}(R^TF) = \sum \hat{\sigma}_i$ and its derivative w.r.t. $F$ is exactly $R$, the total derivative is $2F - 2R$.

### Second term: $\partial/\partial F \frac{\lambda}{2}(J-1)^2 = \lambda(J-1) \cdot \text{cof}(F)$

The derivative of $J = \det(F)$ w.r.t. $F$ is the **cofactor matrix**:

$$
\frac{\partial J}{\partial F} = \text{cof}(F)
$$

Equivalent to $J F^{-T}$ when $F$ is invertible, but the cofactor form is also defined for singular $F$.

The cofactor columns are cross products of the other two columns of $F$:

$$
\text{cof}(F) = [F_1 \times F_2, \; F_2 \times F_0, \; F_0 \times F_1]
$$

**Code**: `compute_cofactor_matrix(F)` uses cross products directly.

### Combined

$$
\boxed{P = 2\mu(F - R) + \lambda(J - 1)\,\text{cof}(F)}
$$

**Code**:

```cpp
Eigen::Matrix3d compute_pk1(...) const {
    auto lame = compute_lame_parameters(youngs_modulus, poisson_ratio);
    auto rotation_data = compute_rotation_data(F);
    double J = F.determinant();
    Eigen::Matrix3d cofactor = compute_cofactor_matrix(F);
    return 2.0 * lame.mu * (F - rotation_data.R) + lame.lambda * (J - 1.0) * cofactor;
}
```

Note: returns $\partial\Psi/\partial F$ (without $V_e$). The rest volume factor is applied in the assembly layer `MaterialEnergy`.

**Verification**:
- $F = I$: $R = I$, $J = 1$ → $P = 0$ ✓
- $F = 2I$: $P = 2\mu I + 28\lambda I$, positive stress resisting expansion ✓

## Hessian: $\partial P / \partial F$ (9×9 matrix)

The Hessian is the derivative of PK1 w.r.t. $F$, stored as a 9×9 matrix after flattening both 3×3 tensors.

Taking the differential of $P$ for a perturbation $dF$:

$$
dP = 2\mu(dF - dR) + \lambda\big(dJ \cdot \text{cof}(F) + (J-1) \cdot d\text{cof}(F)\big)
$$

Three differential quantities are needed: $dR$, $dJ$, and $d\text{cof}(F)$.

### $dJ$: differential of determinant

$$
dJ = \text{cof}(F) : dF = \sum_{ij} \text{cof}(F)_{ij} \, dF_{ij}
$$

**Code**: `cofactor.cwiseProduct(dF).sum()`

### $d\text{cof}(F)$: differential of cofactor

Apply the product rule to $\text{cof}(F) = [F_1 \times F_2, \; F_2 \times F_0, \; F_0 \times F_1]$:

$$
d\text{cof}(F) = \begin{bmatrix}
dF_1 \times F_2 + F_1 \times dF_2, &
dF_2 \times F_0 + F_2 \times dF_0, &
dF_0 \times F_1 + F_0 \times dF_1
\end{bmatrix}
$$

**Code**: `differential_cofactor_matrix(F, dF)`

### $dR$: differential of rotation (most complex)

$R$ is orthogonal, so $dR$ must be $R$ times a skew-symmetric matrix: $dR = R \cdot \hat{\omega}$

(Because $R^T dR + dR^T R = 0$ → $R^T dR$ is skew-symmetric.)

**Deriving $\hat{\omega}$**:

Starting from $R^T F = D\hat{\Sigma}$ (diagonal), differentiate both sides:

$$
dR^T F + R^T dF = d(D\hat{\Sigma})
$$

Substitute $dR = R\hat{\omega}$: $-\hat{\omega} D\hat{\Sigma} + R^T dF = d(D\hat{\Sigma})$

Define $M = R^T dF - dF^T R$ (skew-symmetric), $B = V^T M V$.

The skew-symmetric representation $\tilde{\omega} = V^T \hat{\omega} V$ has upper-triangle entries:

$$
\tilde{\omega}_{ij} = \frac{B_{ij}}{\hat{\sigma}_i + \hat{\sigma}_j}, \qquad i < j
$$

Final result: $dR = UD \cdot \tilde{\omega} \cdot V^T$

**Code**:

```cpp
inline Matrix3d differential_rotation(const Matrix3d& dF, const RotationData& rot) {
    Matrix3d M = rot.R.transpose() * dF - dF.transpose() * rot.R;
    Matrix3d B = rot.V.transpose() * M * rot.V;
    Matrix3d omega_hat = Matrix3d::Zero();
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            double denom = rot.signed_singular_values[i] + rot.signed_singular_values[j];
            if (std::abs(denom) <= 1e-12) continue;  // degenerate guard
            double omega_ij = B(i, j) / denom;
            omega_hat(i, j) = omega_ij;
            omega_hat(j, i) = -omega_ij;
        }
    return rot.UD * omega_hat * rot.V.transpose();
}
```

**Degenerate guard**: when $\hat{\sigma}_i + \hat{\sigma}_j \approx 0$ (nearly collapsed tet), skip that mode — its contribution is negligible.

### Column-by-column 9×9 construction

For $j = 0, \ldots, 8$, set $dF = e_j$ (standard basis reshaped to 3×3), compute $dP$, and flatten into column $j$ of the Hessian:

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

Rationale for column-by-column over closed-form: code directly mirrors the $dP$ definition (easy to verify); reuses helper functions; amenable to per-column FD verification; 9 iterations of 3×3 ops is negligible cost.
