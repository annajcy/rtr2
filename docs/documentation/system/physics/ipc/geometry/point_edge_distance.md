# `point_edge_distance.hpp`

`src/rtr/system/physics/ipc/geometry/point_edge_distance.hpp` implements squared distance from a point to a line segment.

## Theory

Given a point $p$ and segment endpoints $e_0, e_1$, define the edge direction:

$$
u = e_1 - e_0
$$

Project the point onto the infinite line:

$$
\alpha = \frac{(p - e_0)\cdot u}{u \cdot u}
$$

The segment distance has three regions:

1. `Endpoint0` if $\alpha \le 0$
2. `Endpoint1` if $\alpha \ge 1$
3. `EdgeInterior` if $0 < \alpha < 1$

### Local DOF Layout

PE uses 9 local DOFs:

```text
[p_x, p_y, p_z, e0_x, e0_y, e0_z, e1_x, e1_y, e1_z]
```

## Interior Formula

For the interior region, the squared point-line distance is:

$$
s(p, e_0, e_1) =
\frac{\|(e_1 - e_0) \times (p - e_0)\|^2}{\|e_1 - e_0\|^2}
$$

This is the smooth expression evaluated by the current implementation.

## Region Handling in Code

The implementation follows the geometry, not one monolithic formula:

1. validate `p`, `e0`, `e1`
2. reject zero-length edges
3. compute `alpha`
4. branch by region

### Endpoint regions

If the projection lies outside the segment, the nearest point is one endpoint. The code reuses `PointPointDistance`:

```cpp
const auto pp = PointPointDistance::compute({.p0 = input.p, .p1 = input.e0});
detail::embed_distance_result<2, 3>(pp, {0, 1}, result);
```

or the equivalent endpoint-1 mapping.

This means PE does not duplicate the PP derivative logic.

### Interior region

If the projection stays inside the segment, the code packs the 9 local DOFs and evaluates the interior expression through `evaluate_distance_expression<9>(...)`.

## Embedding Rules

Endpoint fallbacks use PP's 6-DOF result and embed it into PE's 9-DOF layout:

- `{0, 1}` means `[p, e0]`
- `{0, 2}` means `[p, e1]`

The unused endpoint gets zero gradient and zero Hessian blocks.

## Degenerate Edge Policy

The current implementation explicitly rejects zero-length edges:

```cpp
if (edge_length_squared <= detail::kMinEdgeLengthSquared) {
    throw std::invalid_argument(...);
}
```

This is intentional. A degenerate edge is not silently treated as "distance zero" because that would hide geometry bugs and contaminate later barrier logic.

## Role in the Larger System

PE is the first composite kernel:

- PT edge regions reuse PE
- EE fallback regions reuse PE
- CCD and tests can consume it through the same `Distance` interface as all other kernels

So PE is where the geometry layer first shows the full design pattern:

- classify first
- evaluate smooth region only after classification
- reuse sub-kernels instead of burying all cases in one expression
