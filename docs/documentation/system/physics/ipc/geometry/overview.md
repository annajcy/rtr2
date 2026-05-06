# IPC Geometry Overview

The `geometry/` directory contains the local geometric distance kernels used by the future IPC contact stack.

This layer is intentionally narrow:

- it accepts only local primitive coordinates
- it returns local dense `distance_squared + gradient + hessian`
- it does not depend on `IPCState`, `IPCSystem`, body metadata, or candidate containers

That makes the same kernels reusable from barrier energy, CCD, and finite-difference tests.

## Current Modules

Current files under `src/rtr/system/physics/ipc/geometry/`:

- `distance_common.hpp`: shared result types, finite checks, embedding helpers, and second-order AutoDiff evaluation
- `distance_concept.hpp`: compile-time interface constraint for distance kernels
- `point_point_distance.hpp`: point-point squared distance
- `point_edge_distance.hpp`: point-segment squared distance with endpoint fallbacks
- `point_triangle_distance.hpp`: point-triangle region classification and composition
- `edge_edge_distance.hpp`: edge-edge region classification, parallel fallback, and composition

## Local DOF Layout

All kernels use a fixed point-major local layout:

| Kernel | Local DOFs |
|---|---|
| PP | `[p0_x, p0_y, p0_z, p1_x, p1_y, p1_z]` |
| PE | `[p_x, p_y, p_z, e0_x, e0_y, e0_z, e1_x, e1_y, e1_z]` |
| PT | `[p_x, p_y, p_z, t0_x, t0_y, t0_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z]` |
| EE | `[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]` |

This ordering is part of the module contract. Barrier assembly, candidate wrappers, and finite-difference tests must all use the same order.

## Architecture Role

`geometry/` sits below the future contact pipeline:

```text
local coordinates
    -> geometry distance kernels
    -> dense local distance / gradient / hessian
    -> barrier/contact assembly
    -> CCD safety checks

same kernels
    -> finite-difference tests
```

More concretely:

- `geometry` evaluates a single primitive pair
- future `contact/barrier` code will consume `distance_squared`, `gradient`, and `hessian`
- future `ccd` code will reuse the same kernels to query local separation along a trial step
- tests use the same interface to compare analytic derivatives against finite differences

## Why the Implementation Is Structured This Way

The current implementation uses three ideas together:

1. **Explicit region classification**
   - PT and EE first decide which geometric region is active
   - this keeps branch logic readable and testable

2. **Kernel reuse + embedding**
   - PT edge and vertex regions reuse PE / PP
   - EE fallback cases reuse PE
   - low-dimensional derivatives are embedded back into the parent 12-DOF layout

3. **Second-order AutoDiff on smooth expressions**
   - once the active region is known, the smooth local expression is evaluated with nested `Eigen::AutoDiffScalar`
   - this yields analytic gradient and Hessian at runtime without falling back to finite differences

This is different from "one giant closed-form PT/EE Hessian function". The current design trades some abstraction for much better maintainability:

- classification bugs are isolated from derivative bugs
- reused sub-kernels stay consistent across PT / EE
- tests can validate the same `Distance` interface everywhere

## Relationship to Later IPC Layers

`geometry/` is not the contact system by itself. It is the mathematical foundation for later layers:

- barrier/contact will turn local squared distance into local barrier energy and sparse global assembly
- CCD will query the same kernels on trial coordinates to bound a safe step size
- solver code will still remain responsible for globalization, PSD handling, and sparse assembly

An important boundary follows from that:

- `geometry/` returns the true local second derivative of the distance expression
- it does **not** perform PSD projection or solver stabilization
- any regularization belongs to barrier or solver layers, not to the distance kernel
