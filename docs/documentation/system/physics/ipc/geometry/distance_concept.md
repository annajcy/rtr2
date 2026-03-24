# `distance_concept.hpp`

`src/rtr/system/physics/ipc/geometry/distance_concept.hpp` defines the compile-time interface checks for IPC local distance kernels.

## Design Motivation

Later IPC contact code will consume several local primitives:

- point-point
- point-edge
- point-triangle
- edge-edge

Those kernels are used in multiple places:

- barrier/contact assembly
- CCD
- finite-difference derivative tests

Without a shared interface, each caller would need kernel-specific adaptation:

- PT might return a struct
- EE might return a tuple
- PE might expose a free function only
- PP might use a different naming convention

That would spread geometry-specific branching into higher-level code.

## Solution: Nested `Input` + `Result` + `compute(...)`

Each kernel defines:

- a nested `Input`
- a nested `Result`
- a static `compute(const Input&)`

The concept constrains only that common shape:

```cpp
template <typename T>
concept Distance = requires(const typename T::Input& input,
                            const typename T::Result& result) {
    typename T::Input;
    typename T::Result;
    { T::compute(input) } -> std::same_as<typename T::Result>;
    { result.distance_squared } -> std::convertible_to<double>;
    result.gradient;
    result.hessian;
};
```

## Why the Concept Does Not Fix the Dimension

Unlike a single global solver interface, local geometry kernels do not all have the same number of DOFs:

- PP has 6
- PE has 9
- PT has 12
- EE has 12

So the concept checks only the existence of:

- `distance_squared`
- `gradient`
- `hessian`

It does not constrain the concrete Eigen sizes.

## Current Kernels Satisfying the Concept

Current coverage:

- `PointPointDistance`
- `PointEdgeDistance`
- `PointTriangleDistance`
- `EdgeEdgeDistance`

This is enough for the current geometry layer and for the next contact/CCD phases built on top of it.

## Benefits of the Uniform Interface

1. **Barrier code can be generic**
   - future code can consume any `Distance` kernel through the same `Input -> Result` protocol

2. **CCD can reuse the same kernels**
   - a local pair evaluator only needs a way to build `Input(alpha)` and call `D::compute(...)`

3. **Tests can share one FD helper**
   - the same finite-difference harness can pack inputs, call `compute(...)`, and compare derivatives

## Difference from `Energy`

`Distance` is deliberately similar in spirit to the existing `Energy` concept, but it targets a different layer:

| | `Energy` | `Distance` |
|---|---|---|
| Operating space | Global DOFs | Local primitive DOFs |
| Return form | Energy + separate gradient / sparse Hessian triplets functions | One local `Result` with dense derivatives |
| Hessian form | Sparse triplets | Dense fixed-size matrix |
| Main consumers | IPC system / Newton solver | Barrier / CCD / derivative tests |

The geometry layer is a local dense evaluation layer, so returning one complete local result is more natural than mirroring the three-function global energy interface.
