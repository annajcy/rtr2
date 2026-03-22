# `tet_material_model_concept.hpp`

`src/rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp` defines the compile-time concept for tet material models.

## Design: Concept + Template, No Virtual Functions

`compute_energy` / `compute_pk1` / `compute_hessian` are called for **every tet** in every Newton iteration — the hottest inner loop. The compiler must be able to inline them.

| Approach | Inline | Intrusiveness | Error messages |
|----------|--------|---------------|----------------|
| Virtual | No | Requires base class | Runtime |
| CRTP | Yes | Requires `Base<Derived>` | Compile-time but hard to read |
| **Concept** | Yes | **Zero intrusion** | Compile-time, clear |

Material types do not need to inherit from anything — they just need to satisfy the interface constraints.

## `TetMaterialModel` Concept

```cpp
template <typename M>
concept TetMaterialModel = requires(const M& material,
                                    const Eigen::Matrix3d& F,
                                    double rest_volume,
                                    double youngs_modulus,
                                    double poisson_ratio) {
    { material.compute_energy(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<double>;
    { material.compute_pk1(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<Eigen::Matrix3d>;
    { material.compute_hessian(F, rest_volume, youngs_modulus, poisson_ratio) }
        -> std::convertible_to<Eigen::Matrix<double, 9, 9>>;
};
```

## What the Three Methods Mean

| Method | Math | Returns | Notes |
|--------|------|---------|-------|
| `compute_energy` | $V_e \cdot \Psi(F)$ | `double` | Total element energy (density × rest volume) |
| `compute_pk1` | $P = \partial\Psi/\partial F$ | `Matrix3d` | PK1 stress (**without** $V_e$) |
| `compute_hessian` | $\partial^2\Psi/\partial F^2$ | `Matrix<9,9>` | F-space Hessian ($F$ flattened to 9D) |

Note: `compute_energy` includes $V_e$, but `compute_pk1` and `compute_hessian` do not — $V_e$ is applied in the assembly layer `MaterialEnergy`.

## Parameter Reference

| Parameter | Type | Meaning |
|-----------|------|---------|
| `F` | `Matrix3d` | Deformation gradient, $F = D_s D_m^{-1}$ |
| `rest_volume` | `double` | Reference volume $V_e = \|\det(D_m)\| / 6$ |
| `youngs_modulus` | `double` | Young's modulus $E$ (stiffness) |
| `poisson_ratio` | `double` | Poisson ratio $\nu \in (-1, 0.5)$ |

The material model only cares about "given $F$, return energy/stress/Hessian". Building $F$ (extracting $D_s$ from global DOFs) and mapping the result (PK1 → nodal forces) is the assembly layer `MaterialEnergy`'s responsibility.

## Current Implementation

The only concrete material: `FixedCorotatedMaterial` (see `tet_fixed_corotated.hpp`).

```cpp
static_assert(TetMaterialModel<FixedCorotatedMaterial>);
```

## Extension

Adding a new material (e.g., Neo-Hookean, StVK) only requires defining a struct with three const methods. No inheritance, registration, or changes to existing code needed.

If runtime material selection is needed later (e.g., editor dropdown), use `std::variant<FixedCorotated, NeoHookean, ...>` + `std::visit` — the inner loop still gets template inlining.
