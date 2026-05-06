# `point_point_distance.hpp`

`src/rtr/system/physics/ipc/geometry/point_point_distance.hpp` implements the smallest local distance kernel: squared distance between two points.

## Theory

For two points $p_0, p_1 \in \mathbb{R}^3$:

$$
s(p_0, p_1) = \|p_0 - p_1\|^2
$$

This is the base case for the whole geometry layer.

### Local DOF Layout

PP uses 6 local DOFs:

```text
[p0_x, p0_y, p0_z, p1_x, p1_y, p1_z]
```

### Gradient

Let $d = p_0 - p_1$. Then

$$
\nabla_{p_0} s = 2d, \qquad \nabla_{p_1} s = -2d
$$

so the full local gradient is:

$$
\nabla s =
\begin{bmatrix}
2(p_0 - p_1) \\
-2(p_0 - p_1)
\end{bmatrix}
$$

### Hessian

The Hessian is constant:

$$
\nabla^2 s =
\begin{bmatrix}
2I & -2I \\
-2I & 2I
\end{bmatrix}
$$

There are no region changes and no degeneracy branches.

## API

```cpp
struct PointPointDistance {
    struct Input {
        Eigen::Vector3d p0;
        Eigen::Vector3d p1;
    };

    using Result = PointPointDistanceResult;

    static Result compute(const Input& input);
};
```

There is also a thin convenience wrapper:

```cpp
PointPointDistanceResult point_point_distance(const Eigen::Vector3d& p0,
                                             const Eigen::Vector3d& p1);
```

## Implementation

The implementation is intentionally minimal:

1. validate both points are finite
2. pack them into a 6-DOF local vector
3. evaluate $\|p_0 - p_1\|^2$ through `evaluate_distance_expression<6>(...)`

Core expression:

```cpp
const auto p0 = x.template segment<3>(0);
const auto p1 = x.template segment<3>(3);
return detail::squared_norm(p0 - p1);
```

Although the gradient and Hessian are available in closed form, the code still routes through the shared AutoDiff helper so that all kernels follow the same derivative pipeline.

## Why PP Has No Epsilon Logic

PP is the one kernel that needs no geometric classification:

- there is no projection parameter
- there is no endpoint vs interior distinction
- there is no degeneracy branch

That makes PP the cleanest reference kernel for derivative tests and for validating the shared derivative machinery.

## Role in the Larger System

PP is not just a toy primitive:

- PE endpoint regions fall back to PP
- PT vertex regions fall back to PP
- degenerate PT cases may compare PP candidates
- it is the smallest FD baseline for validating local analytic derivatives
