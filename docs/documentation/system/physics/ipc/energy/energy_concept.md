# `energy_concept.hpp`

`src/rtr/system/physics/ipc/energy/energy_concept.hpp` defines the compile-time interface checks for IPC energy modules.

## Design Motivation

The IPC solver's total energy is a sum of independent terms:

$$
E(x) = E_{inertial}(x) + E_{gravity}(x) + E_{elastic}(x) + \ldots
$$

Each energy term must provide energy / gradient / Hessian triplets computation. But different terms need different inputs:

- `InertialEnergy` needs `x`, `x_hat`, `mass_diag`, `dt`
- `GravityEnergy` needs `x`, `mass_diag`, `gravity`
- `MaterialEnergy<M>` needs `TetBody`, `x`, `Material`

Forcing all parameters into a single argument list would be bloated and fragile.

## Solution: Nested `Input` + Concept

Each energy type defines its own `Input` struct bundling the required parameters. The `Energy` concept constrains only the interface shape:

```cpp
template <typename T>
concept Energy = requires(const typename T::Input& input,
                          Eigen::VectorXd& gradient,
                          std::vector<Eigen::Triplet<double>>& triplets) {
    typename T::Input;
    { T::compute_energy(input) } -> std::convertible_to<double>;
    { T::compute_gradient(input, gradient) } -> std::same_as<void>;
    { T::compute_hessian_triplets(input, triplets) } -> std::same_as<void>;
};
```

## `Input` Per Energy Term

| Energy | `Input` contents | Hessian |
|--------|-----------------|---------|
| `InertialEnergy` | `x`, `x_hat`, `mass_diag`, `dt` | Diagonal, positive definite |
| `GravityEnergy` | `x`, `mass_diag`, `gravity` | Zero (no-op) |
| `MaterialEnergy<M>` | `TetBody`, `x`, `Material` | Sparse, per-tet 12×12 blocks |

## Benefits of the Uniform Interface

1. **Generic assembly**: future code can write `template <Energy E> void assemble(...)` to handle all terms uniformly
2. **Zero-Hessian terms need no special case**: `GravityEnergy`'s `compute_hessian_triplets` is a no-op, but the concept is satisfied
3. **New energy terms just satisfy the concept**: adding barrier / friction only requires defining `Input` + three methods

## Difference from `TetMaterialModel`

| | `Energy` | `TetMaterialModel` |
|---|---------|---------------------|
| Operating space | Global DOFs ($3n$ vector) | Element-local ($3 \times 3$ $F$) |
| Hessian format | Triplets (sparse) | Dense $9 \times 9$ matrix |
| Bridge | Called directly by solver | Bridged via `MaterialEnergy` |
