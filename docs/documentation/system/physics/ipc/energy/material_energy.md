# `material_energy.hpp`

`src/rtr/system/physics/ipc/energy/material_energy.hpp` bridges a tet-local `TetMaterialModel` back into the global IPC energy interface.

## Role

Material models work in $F$-space (3×3 matrices). The solver works in global DOF space ($3n$-dimensional vectors). `MaterialEnergy<Material>` converts between the two:

- Material provides: $\Psi(F)$, $P = \partial\Psi/\partial F$ (3×3), $\partial^2\Psi/\partial F^2$ (9×9)
- Solver needs: total energy scalar, global DOF gradient (3n), global Hessian triplets

The conversion involves three levels of chain rule:

$$
E(x) = \sum_e V_e \cdot \Psi(F_e), \quad F_e = D_s(x) \cdot D_m^{-1}
$$

## Helper Functions

### `build_Ds`: extract current-configuration edge vectors from global DOFs

For tet $(v_0, v_1, v_2, v_3)$ and global position vector $x$:

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

`dof_offset` is the body's starting position in the global DOF vector (for multi-body, the second body starts at `3 * vertex_count_of_body_0`).

### `compute_shape_gradients`: compute $\nabla N_a$

Shape function gradients are determined by $D_m^{-1}$:

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

### `build_dFdx_matrix`: $\partial \text{vec}(F) / \partial x_a$ (9×3 matrix)

This matrix encodes "how moving node $a$ changes $F$". The derivation starts from $F = D_s \cdot D_m^{-1}$.

#### Deriving $\delta F = \delta x_a \otimes \nabla N_a$

Recall $D_s = [x_1 - x_0, \; x_2 - x_0, \; x_3 - x_0]$, so $F = D_s \cdot D_m^{-1}$.

**For $a \in \{1, 2, 3\}$**: perturbing $x_a$ by $\delta x_a$ (a 3D vector) changes only column $(a{-}1)$ of $D_s$:

$$
\delta D_s = \delta x_a \cdot e_{a-1}^T
$$

where $e_{a-1}$ is a standard basis vector ($e_0 = [1,0,0]^T$, etc.). The outer product $\delta x_a \cdot e_{a-1}^T$ is a rank-1 3×3 matrix: only column $(a{-}1)$ is nonzero, equal to $\delta x_a$.

Substituting into $F = D_s \cdot D_m^{-1}$:

$$
\delta F = \delta D_s \cdot D_m^{-1} = (\delta x_a \cdot e_{a-1}^T) \cdot D_m^{-1} = \delta x_a \cdot \underbrace{(e_{a-1}^T D_m^{-1})}_{\text{1×3 row vector}}
$$

$e_{a-1}^T D_m^{-1}$ is row $(a{-}1)$ of $D_m^{-1}$, whose transpose is column $(a{-}1)$ of $D_m^{-T}$. Recalling the earlier definition $\nabla N_a = D_m^{-T} e_{a-1}$:

$$
\boxed{\delta F = \delta x_a \otimes \nabla N_a}
$$

This is a 3×3 outer product: $(\delta F)_{ij} = (\delta x_a)_i \cdot (\nabla N_a)_j$.

**For $a = 0$**: $x_0$ appears in all three columns of $D_s$ (as the subtracted term), so moving $x_0$ changes all three columns simultaneously:

$$
\delta D_s = [-\delta x_0, \; -\delta x_0, \; -\delta x_0]
$$

$$
\delta F = \delta D_s \cdot D_m^{-1} = -\delta x_0 \cdot [1,1,1] \cdot D_m^{-1} = \delta x_0 \otimes \underbrace{(-D_m^{-T}(e_0 + e_1 + e_2))}_{\nabla N_0}
$$

This is exactly $\delta x_0 \otimes \nabla N_0$, since $\nabla N_0 = -(\nabla N_1 + \nabla N_2 + \nabla N_3)$.

So the formula holds uniformly for all 4 vertices: $\delta F = \delta x_a \otimes \nabla N_a$.

**Numerical verification**: take the reference tet $D_m = I$, $\nabla N_1 = (1,0,0)$. Perturb $x_1$ by $\delta x_1 = (0, \epsilon, 0)$:

$$
\delta F = \begin{bmatrix}0\\\epsilon\\0\end{bmatrix} \otimes \begin{bmatrix}1\\0\\0\end{bmatrix} = \begin{bmatrix}0&0&0\\\epsilon&0&0\\0&0&0\end{bmatrix}
$$

Only $\delta F_{10} = \epsilon$ is nonzero — pushing $x_1$ (the x-direction vertex) in the y direction causes $F_{10}$ to change. Direct computation via $\delta D_s$ gives the same result. ✓

#### Flattening to a 9×3 matrix

Given $\delta F = \delta x_a \otimes \nabla N_a$, we now write it in matrix-multiply form. Flattened:

$$
\text{vec}(\delta F) = \underbrace{\frac{\partial \text{vec}(F)}{\partial x_a}}_{9 \times 3} \cdot \delta x_a
$$

Since $(\delta F)_{ij} = (\delta x_a)_i \cdot (\nabla N_a)_j$, in Eigen's column-major layout ($F_{ij} \to$ index $i + 3j$):

$$
\text{dFdx}_{(i+3j),\, i} = (\nabla N_a)_j
$$

For $\nabla N_a = (g_0, g_1, g_2)$:

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

Each group of 3 rows corresponds to one column of $F$; within each group, it's $(\nabla N_a)_j \cdot I_3$.

```cpp
inline Matrix<double, 9, 3> build_dFdx_matrix(const Vector3d& grad_N) {
    Matrix<double, 9, 3> dFdx = Matrix<double, 9, 3>::Zero();
    for (int col = 0; col < 3; ++col)        // F column index j
        for (int row = 0; row < 3; ++row)     // F row index i = δx_a component
            dFdx(row + 3 * col, row) = grad_N[col];
    return dFdx;
}
```

## Energy Assembly

$$
E_{elastic}(x) = \sum_e V_e \cdot \Psi(F_e)
$$

Loop over all tets, compute $F$, call material's `compute_energy`, accumulate.

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

Note: `compute_energy` already returns $V_e \cdot \Psi(F)$ (rest_volume is multiplied inside the material).

## Gradient Assembly

### Theory

Per-node gradient contribution:

$$
\frac{\partial E_e}{\partial x_a} = V_e \cdot P_e \cdot \nabla N_a
$$

This is a 3D vector. When multiple tets share a vertex, gradients accumulate naturally (`+=` mode).

### Code

```cpp
static void compute_gradient(const Input& input, VectorXd& gradient) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        Matrix3d F = Ds * Dm_inv[t];
        Matrix3d P = input.material.compute_pk1(F, V, E, nu);
        auto grad_N = compute_shape_gradients(Dm_inv[t]);

        for (int a = 0; a < 4; ++a) {
            Vector3d local_grad = V * P * grad_N[a];
            gradient.segment<3>(dof_offset + 3 * tet[a]) += local_grad;
        }
    }
}
```

`P * grad_N[a]` is matrix-vector (3×3 × 3×1 = 3×1). $P$ encodes "stress in each reference direction"; $\nabla N_a$ encodes "which reference direction node $a$ influences". Their product = "force node $a$ receives from the stress field".

### Numerical verification

Reference tet ($D_m = I$), $F = \text{diag}(2,1,1)$, $\mu=1, \lambda=1, V_e=1/6$:

$P = \text{diag}(2, 0, -1)$, $\nabla N_1 = (1, 0, 0)$

$$
\text{grad}_{v_1} = \tfrac{1}{6}(2, 0, 0)^T
$$

$v_1$ has positive gradient along x (further stretching increases energy). ✓

## Hessian Assembly

### Theory

**Goal**: compute tet $e$'s contribution to the global Hessian block $(v_a, v_b)$ — a 3×3 matrix.

Starting from the gradient and differentiating again:

$$
\frac{\partial^2 E_e}{\partial x_a \partial x_b} = V_e \cdot \frac{\partial (P \cdot \nabla N_a)}{\partial x_b}
$$

Since $\nabla N_a$ is constant, this becomes $V_e \cdot \frac{\partial P}{\partial x_b} \cdot \nabla N_a$. Working with $\partial P / \partial x_b$ directly (a rank-4 tensor) is unwieldy.

**Key trick**: use flattening + chain rule:

$$
\boxed{H_{ab}^{(e)} = V_e \cdot \text{dFdx}_a^T \cdot H_F \cdot \text{dFdx}_b}
$$

where $H_F = \partial^2\Psi/\partial F^2$ is the material's 9×9 Hessian and $\text{dFdx}_a$ is the 9×3 Jacobian.

**Dimension check**: $(3 \times 9) \cdot (9 \times 9) \cdot (9 \times 3) = 3 \times 3$ ✓

Each tet has 4 vertices → $4 \times 4 = 16$ blocks of 3×3 → $144$ scalar triplets.

### Symmetrization

The material's `compute_hessian` may not be exactly symmetric due to numerical precision. Explicit symmetrization before assembly:

```cpp
hessian_F = 0.5 * (hessian_F + hessian_F.transpose());
```

This ensures the global Hessian is symmetric (required by `SimplicialLDLT`).

### Code

```cpp
static void compute_hessian_triplets(const Input& input,
                                     vector<Triplet<double>>& triplets) {
    for (size_t t = 0; t < input.body.tet_count(); ++t) {
        Matrix3d F = Ds * Dm_inv[t];
        double V = rest_volumes[t];

        Matrix<double, 9, 9> H_F = input.material.compute_hessian(F, V, E, nu);
        H_F = 0.5 * (H_F + H_F.transpose());

        auto grad_N = compute_shape_gradients(Dm_inv[t]);
        array<Matrix<double, 9, 3>, 4> dFdx;
        for (int a = 0; a < 4; ++a)
            dFdx[a] = build_dFdx_matrix(grad_N[a]);

        for (int a = 0; a < 4; ++a) {
            Index row_base = dof_offset + 3 * tet[a];
            for (int b = 0; b < 4; ++b) {
                Index col_base = dof_offset + 3 * tet[b];
                Matrix3d block = V * dFdx[a].transpose() * H_F * dFdx[b];
                for (int i = 0; i < 3; ++i)
                    for (int j = 0; j < 3; ++j)
                        triplets.emplace_back(row_base + i, col_base + j, block(i, j));
            }
        }
    }
}
```

### Data flow

```
For each tet e:
    x (3n global DOFs)
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
    ├── compute_shape_gradients() ──→ ∇N_a (4 × 3D vectors)
    │                          │            │
    │                          │    build_dFdx_matrix()
    │                          │            │
    │                          │            ▼
    │                          │    dFdx_a (4 × 9×3)
    │                          │            │
    │                          ▼            ▼
    │               gradient += V*P*∇N_a   H_ab = V * dFdx_a^T * H_F * dFdx_b
    │                                       │
    │                                       ▼
    │                              triplets.emplace_back(...)
    │                              (16 blocks × 9 entries = 144 triplets per tet)
```

### Numerical example

Reference tet ($D_m = I$), $F = I$, $\mu = 1, \lambda = 1, V_e = 1/6$.

$\nabla N_1 = (1,0,0)$ → $\text{dFdx}_1$ is nonzero only at $(0,0), (1,1), (2,2)$, selecting the top-left 3×3 block of $H_F$.

At $F = I$, this block $\approx \text{diag}(2\mu + \lambda, 2\mu, 2\mu) = \text{diag}(3, 2, 2)$.

$$
H_{11} \approx \tfrac{1}{6}\text{diag}(3, 2, 2)
$$

Physical meaning: stretching $v_1$ along x has the highest stiffness (stretch mode = $\mu + \lambda$); y/z directions are pure shear ($= \mu$). ✓

## Computational Cost

| Operation | Cost |
|-----------|------|
| `build_Ds` + `F = Ds * Dm_inv` | $O(27)$ |
| `compute_pk1` (includes SVD) | $O(100+)$ |
| `compute_hessian` (9 cols × SVD diff) | $O(900+)$ |
| `dFdx_a^T * H_F * dFdx_b` (16 blocks) | $O(16 \times 81)$ |
| Triplet writes (144) | $O(144)$ |

Gradient assembly is an order of magnitude cheaper than Hessian assembly (no 9×9 Hessian, no 16 matrix multiplies).
