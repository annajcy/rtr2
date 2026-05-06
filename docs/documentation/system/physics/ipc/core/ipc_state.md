# `ipc_state.hpp`

`src/rtr/system/physics/ipc/core/ipc_state.hpp` defines `IPCState`, the global DOF container for future FEM/IPC solves.

## Why a Global Vector Exists

The continuous system starts from a displacement field `x(X, t)`. Linear FEM replaces that infinite-dimensional field with nodal samples and shape functions:

$$
\hat{x}(X) = \sum_{a=1}^{N} x_a N_a(X)
$$

where `x_a \in \mathbb{R}^3` is the position of vertex `a`. Once the field is discretized, all nodal unknowns are stacked into one global vector:

$$
\mathbf{x} =
\begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_N
\end{bmatrix}
\in \mathbb{R}^{3N}
$$

`IPCState::x` stores exactly that vector.

## Layout

`IPCState` uses a vertex-major layout:

```text
x = [x0x, x0y, x0z, x1x, x1y, x1z, ..., x(N-1)z]^T
```

So vertex `i` starts at `3 * i`, which is why `position(i)` is implemented as `x.segment<3>(3 * i)`.

The same layout is shared by:

- `x_prev`
- `v`
- `mass_diag`

## Fields

- `x`: current nodal positions
- `x_prev`: previous-step nodal positions
- `v`: nodal velocities
- `mass_diag`: lumped diagonal mass, stored directly as a `3N` vector

If vertex `i` has scalar mass `m_i`, the diagonal mass matrix is represented as

$$
\mathbf{M} = \mathrm{diag}(m_1, m_1, m_1, m_2, m_2, m_2, \ldots, m_N, m_N, m_N)
$$

This is why `mass_diag` repeats the same scalar mass three times per vertex.

## API Shape

`resize(vertex_count)` zeros all four vectors and resizes them to `3 * vertex_count`.

`vertex_count()` and `position(i)` rely on debug-time `assert` checks instead of exceptions. The container is meant to sit on hot paths, so `resize()` establishes the invariant and accessors stay lightweight in release builds.

## Why This Representation Matters

Later energies and derivatives will all target the same global variable. A typical inertial term already has the form

$$
E_I(\mathbf{x}) = \frac{1}{2h^2}(\mathbf{x} - \hat{\mathbf{x}})^T \mathbf{M} (\mathbf{x} - \hat{\mathbf{x}})
$$

With a lumped diagonal mass, this becomes a simple per-DOF sum, which is why `IPCState` stores vectors instead of building a sparse mass matrix at this stage.
