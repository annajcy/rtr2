# `point_triangle_distance.hpp`

`src/rtr/system/physics/ipc/geometry/point_triangle_distance.hpp` implements squared distance from a point to a triangle, including region classification, degeneracy handling, and derivative composition.

This is the most important local primitive for the upcoming IPC contact barrier.

## Theory: Seven Closest Regions

For a point $p$ and triangle vertices $(t_0, t_1, t_2)$, the nearest point on the triangle must lie in one of seven regions:

1. face interior
2. edge `t0-t1`
3. edge `t0-t2`
4. edge `t1-t2`
5. vertex `t0`
6. vertex `t1`
7. vertex `t2`

The implementation reflects exactly this geometric decomposition.

### Local DOF Layout

PT uses 12 local DOFs:

```text
[p_x, p_y, p_z, t0_x, t0_y, t0_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z]
```

## Region Classification

The code first computes:

$$
ab = t_1 - t_0, \qquad ac = t_2 - t_0
$$

and then uses the standard dot-product tests on:

- `ap = p - t0`
- `bp = p - t1`
- `cp = p - t2`

to decide whether the closest point is:

- a vertex region
- an edge region
- or the face interior

In code, those tests appear through the scalars `d1` through `d6`, and the signed area-style expressions `va`, `vb`, `vc`.

The branch order matters:

1. check `Vertex0`
2. check `Vertex1`
3. check `Edge01`
4. check `Vertex2`
5. check `Edge02`
6. check `Edge12`
7. otherwise the point projects to the face interior

This order matches the nearest-region partition and keeps the logic mutually exclusive.

## Three Evaluation Paths

### Face interior

If the nearest point lies in the triangle interior, the current implementation evaluates point-plane squared distance:

$$
n = (t_1 - t_0) \times (t_2 - t_0)
$$

$$
s =
\frac{\big((p - t_0)\cdot n\big)^2}{\|n\|^2}
$$

The code intentionally uses the **unnormalized** normal. This avoids introducing an extra explicit unit-normal normalization chain into the handwritten formula.

### Edge regions

If the nearest point lies on one triangle edge, PT reuses `PointEdgeDistance`:

- `Edge01` uses `{p, t0, t1}`
- `Edge02` uses `{p, t0, t2}`
- `Edge12` uses `{p, t1, t2}`

The resulting 9-DOF derivative is then embedded back into the 12-DOF PT layout.

### Vertex regions

If the nearest point is one triangle vertex, PT reuses `PointPointDistance`:

- `Vertex0` uses `{p, t0}`
- `Vertex1` uses `{p, t1}`
- `Vertex2` uses `{p, t2}`

Again, the lower-dimensional derivative is embedded into the 12-DOF parent layout.

## Degenerate Triangle Policy

The code first measures triangle area through:

$$
\|(t_1 - t_0) \times (t_2 - t_0)\|^2
$$

If the area is below `kMinTriangleAreaSquared`, the triangle is treated as degenerate and the face formula is skipped.

Current fallback strategy in `compute_degenerate_triangle(...)`:

1. evaluate PP to each vertex
2. evaluate PE on each non-degenerate edge
3. choose the candidate with the smallest `distance_squared`

This is a geometry-preserving downgrade:

- if the triangle collapses to a segment, PE still makes geometric sense
- if it collapses to a point, PP still makes geometric sense

The implementation never tries to force a face-interior formula through a near-zero-area triangle.

## Embedding Back Into PT Layout

Because PT composes PP and PE, embedding is part of the design, not an implementation accident.

Examples:

- PP `{p, t0}` maps to PT point indices `{0, 1}`
- PE `{p, t1, t2}` maps to PT point indices `{0, 2, 3}`

The embedded result fills only the participating blocks; the non-participating vertex receives zero entries.

## Current Derivative Strategy

An important implementation fact:

- **region classification is handwritten geometric logic**
- **smooth-region gradient and Hessian are produced by second-order AutoDiff**

So the code is not doing finite differencing, but it is also not hard-coding one huge symbolic PT Hessian for every region.

That split is deliberate:

- classification is clearer when written as explicit geometry tests
- derivatives are safer when generated from the active smooth expression
- reused PE / PP regions automatically inherit the same derivative behavior as their source kernels

## Tests

The current tests cover:

- all seven named regions
- a degenerate triangle case
- finite-difference checks on representative smooth cases

The FD tests intentionally avoid active-set switching boundaries. A central difference around a region boundary is not a reliable validation target for a piecewise distance kernel, even if each smooth branch is correct.

## Role in Later IPC Contact

PT is one of the two main contact primitives that future barrier and CCD code will consume directly.

That is why this file does more than just evaluate a scalar distance:

- it returns a full dense local derivative bundle
- it keeps a stable 12-DOF ordering
- it makes degeneracy handling explicit
- it exposes region information for debugging and testing
