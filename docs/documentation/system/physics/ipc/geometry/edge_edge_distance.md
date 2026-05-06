# `edge_edge_distance.hpp`

`src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp` implements squared distance between two line segments.

Together with PT, this is the other main primitive that later IPC barrier and CCD code will use directly.

## Theory

For segments $(ea_0, ea_1)$ and $(eb_0, eb_1)$, the closest pair falls into one of three broad categories:

1. both closest points lie in the segment interiors
2. one endpoint is closest to the other segment
3. a degenerate / near-degenerate configuration must be handled by fallback logic

The numerically dangerous case is not generic skew edges. It is:

- parallel edges
- nearly parallel edges
- very short edges

That is why the implementation is organized around safe classification and fallback.

### Local DOF Layout

EE uses 12 local DOFs:

```text
[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]
```

## Interior-Interior Test

Let:

$$
u = ea_1 - ea_0,\qquad v = eb_1 - eb_0,\qquad w_0 = ea_0 - eb_0
$$

Then the standard closest-line parameters are computed from:

$$
a = u\cdot u,\quad b = u\cdot v,\quad c = v\cdot v,\quad d = u\cdot w_0,\quad e = v\cdot w_0
$$

$$
s = \frac{be - cd}{ac - b^2}, \qquad
t = \frac{ae - bd}{ac - b^2}
$$

If both `s` and `t` lie strictly inside `(0, 1)`, the kernel treats the active region as `InteriorInterior`.

### Interior smooth expression

For that case, the current implementation evaluates segment-line separation through the unnormalized cross-product form:

$$
n = (ea_1 - ea_0) \times (eb_1 - eb_0)
$$

$$
s =
\frac{\big((ea_0 - eb_0)\cdot n\big)^2}{\|n\|^2}
$$

As in PT face evaluation, the code uses the unnormalized normal-like quantity and lets AutoDiff handle the derivatives of the smooth expression.

## Parallel / Near-Parallel Handling

Before trusting the interior formula, the code checks:

$$
\|(ea_1 - ea_0) \times (eb_1 - eb_0)\|^2
$$

If this quantity is below `kParallelThreshold`, the edges are treated as parallel or near-parallel and the kernel skips the interior formula entirely.

This matters because the interior parameter solve becomes numerically fragile when the two directions are almost linearly dependent.

## Fallback Design

The current fallback path is explicit and geometric:

1. evaluate `ea0 -> edge B`
2. evaluate `ea1 -> edge B`
3. evaluate `eb0 -> edge A`
4. evaluate `eb1 -> edge A`
5. choose the smallest `distance_squared`

Each of those candidates is a `PointEdgeDistance` evaluation embedded back into EE's 12-DOF layout.

So fallback derivatives are not re-derived in EE. They come from the already-tested PE kernel.

## Zero-Length Edge Policy

The current implementation rejects zero-length input edges:

```cpp
if (edge_a_length_squared <= detail::kMinEdgeLengthSquared ||
    edge_b_length_squared <= detail::kMinEdgeLengthSquared) {
    throw std::invalid_argument(...);
}
```

This is stricter than PT's degenerate-triangle downgrade because a true EE kernel without two valid edges has no clean "segment-segment" meaning left.

## Current Derivative Strategy

As with PT:

- region selection is handwritten geometric logic
- the active smooth expression uses second-order AutoDiff
- fallback regions reuse lower-level kernels instead of duplicating derivative code

That gives EE a practical split:

- explicit control over dangerous geometric cases
- analytic derivatives on smooth branches
- compositional reuse for degenerate branches

## Tests

The current tests cover:

- interior-interior
- endpoint-on-edge fallback
- endpoint-endpoint style fallback through the same candidate machinery
- parallel
- nearly parallel
- short-edge behavior

Finite-difference checks are run only on representative smooth cases, not on active-set switching boundaries.

## Role in Later IPC Contact

EE is a core contact primitive because triangle meshes and collision surfaces frequently produce edge-edge proximity pairs.

For that reason, the file does more than compute a scalar minimum:

- it keeps the local 12-DOF ordering stable
- it preserves region/debug information
- it makes parallel fallback a first-class branch
- it returns local dense derivatives ready for later barrier assembly
