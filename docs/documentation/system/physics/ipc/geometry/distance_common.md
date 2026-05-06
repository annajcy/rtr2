# `distance_common.hpp`

`src/rtr/system/physics/ipc/geometry/distance_common.hpp` contains the shared building blocks used by all current distance kernels.

This file is not a public "geometry algorithm" by itself. Its job is to provide the infrastructure that lets PP / PE / PT / EE share the same result shape and derivative machinery.

## `DistanceResult<Dofs>`

The core type is:

```cpp
template <int Dofs>
struct DistanceResult {
    double distance_squared;
    Eigen::Matrix<double, Dofs, 1> gradient;
    Eigen::Matrix<double, Dofs, Dofs> hessian;
};
```

This gives each kernel a fixed-size local return object:

- one scalar `distance_squared`
- one dense local gradient
- one dense local Hessian

The fixed dimension matters because IPC contact is fundamentally local:

- PP uses 6 DOFs
- PE uses 9 DOFs
- PT and EE use 12 DOFs

The barrier layer will later map these dense blocks into global sparse assembly.

## Shared Support in `detail`

### Finite-input and finite-output checks

`require_finite_point(...)` validates local coordinates before any geometric work starts.

`require_finite_result(...)` validates the final scalar / gradient / Hessian before returning to callers.

This follows the same philosophy used elsewhere in the IPC code:

- invalid local geometry should fail early
- geometry kernels should not silently inject NaNs into barrier or solver code

### `embed_distance_result(...)`

PT and EE reuse lower-dimensional kernels. For example:

- PT edge regions call PE
- PT vertex regions call PP
- EE fallback regions call PE

Those sub-kernels return derivatives in their own local layout. `embed_distance_result(...)` writes them back into the parent layout by copying the participating `3 x 3` blocks into the correct positions of the larger gradient and Hessian.

Conceptually:

```text
PP / PE local derivative
    -> point index map
    -> copy into PT / EE dense block
```

This keeps reuse explicit and avoids re-deriving the same endpoint/edge formulas multiple times.

### `evaluate_distance_expression(...)`

The most important helper is `evaluate_distance_expression(...)`.

Its input is:

- one packed local DOF vector
- one smooth scalar expression of those DOFs

It then evaluates that expression with nested `Eigen::AutoDiffScalar` to recover:

- the value
- the first derivative
- the second derivative

The implementation uses a two-level AutoDiff scalar:

```cpp
using InnerScalar = Eigen::AutoDiffScalar<FirstDerivative>;
using Scalar = Eigen::AutoDiffScalar<SecondDerivative>;
```

The outer level stores first derivatives whose values are themselves differentiable inner scalars. That is what lets the code recover a dense Hessian without hand-writing all second derivatives.

This is still an **analytic derivative path at runtime**. It is not finite differencing. The kernel evaluates a symbolic expression through automatic differentiation.

## Why `cross`, `squared_norm`, and Thresholds Live Here

The same tiny geometric utilities appear in multiple kernels:

- `squared_norm(...)` keeps scalar expressions generic
- `cross(...)` avoids depending on a wider Eigen geometry module for these 3D kernels
- numeric thresholds centralize degeneracy policy

Current thresholds:

| Constant | Meaning |
|---|---|
| `kMinEdgeLengthSquared` | reject effectively zero-length edges |
| `kMinTriangleAreaSquared` | treat nearly zero-area triangles as degenerate |
| `kParallelThreshold` | treat very small edge-edge cross products as parallel / near-parallel |

These constants are not "physics parameters". They are geometric robustness guards for choosing a safe evaluation path.

## Hessian Policy

An important architectural rule is enforced here indirectly:

- the geometry layer returns the true local second derivative of the active distance expression
- it does **not** project the Hessian to PSD
- it does **not** regularize for solver stability

That is the correct split for IPC:

$$
H_{barrier} = b''(s)\nabla s \nabla s^T + b'(s)\nabla^2 s
$$

Any stabilization or PSD projection belongs to the barrier or solver layer, not to `distance_common.hpp`.
