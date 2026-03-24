# `collision_candidates.hpp`

`src/rtr/system/physics/ipc/contact/collision_candidates.hpp` implements the first broad-phase layer on top of `CollisionMesh`.

The current goal is correctness and stable behavior, not acceleration. Candidate generation is therefore brute force, but it already encodes the primitive combinations and filtering rules that later barrier / CCD code depend on.

## Candidate Types

The file defines two primitive-pair records:

```cpp
struct PTCandidate {
    std::size_t point_vertex_idx;
    std::size_t triangle_idx;
};

struct EECandidate {
    std::size_t edge_a_idx;
    std::size_t edge_b_idx;
};
```

and one aggregate container:

```cpp
struct CollisionCandidates {
    std::vector<PTCandidate> pt_candidates;
    std::vector<EECandidate> ee_candidates;
};
```

The candidates only store primitive indices into `CollisionMesh`. Ownership and coordinate lookup stay in the mesh instead of being duplicated here.

## Day 2 Broad-Phase Scope

The current implementation intentionally generates only the combinations needed by the first tet-vs-static-obstacle pipeline:

1. deformable point vs obstacle triangle
2. obstacle point vs deformable triangle
3. deformable edge vs obstacle edge

This is enough to cover vertex-face and edge-edge proximity without opening self-contact or obstacle-obstacle paths too early.

## Deterministic Enumeration

The entry point is:

```cpp
CollisionCandidates build_collision_candidates(
    const CollisionMesh& mesh,
    const IPCState& state,
    const BroadPhaseConfig& config = {}
);
```

The loops are intentionally ordered:

1. deformable vertices × obstacle triangles
2. obstacle vertices × deformable triangles
3. deformable edges × obstacle edges

All loops iterate over pre-sorted primitive index lists stored in `CollisionMesh`, so the output order is stable for a fixed mesh. That matters for testing and debugging.

## Filtering Rules

Broad phase is not allowed to emit every cross-product pair. It must remove topologically invalid or currently unsupported cases.

### Point-Triangle

A PT pair is dropped when:

- the point already belongs to that triangle
- the point and triangle belong to the same body
- both sides are deformable or both sides are obstacles

The first rule avoids trivial self-incidence. The other two rules encode the current Day 2 contact scope.

### Edge-Edge

An EE pair is dropped when:

- the two edges belong to the same body
- they share any endpoint
- both edges are on the same side category

The shared-endpoint rule prevents edge pairs that are topologically adjacent rather than geometrically separate.

## Optional AABB Prefilter

`BroadPhaseConfig` exposes an optional conservative prefilter:

```cpp
struct BroadPhaseConfig {
    bool enable_aabb_prefilter{false};
    double aabb_padding{0.0};
};
```

When enabled:

- PT keeps a pair only if the point AABB overlaps the triangle AABB
- EE keeps a pair only if the two edge AABBs overlap

Padding expands each box uniformly on all axes:

$$
\text{AABB}_{\text{padded}} = [x_{\min} - p,\ x_{\max} + p]
$$

This prefilter is still conservative. It can keep too many candidates, but it should not remove a truly possible contact when used with a reasonable padding.

## Why `IPCState` Is Already in the Interface

Even though the default Day 2 path can be pure brute force, the function already takes `IPCState` because all optional spatial filtering depends on current deformable coordinates.

That keeps the API stable when the caller later enables:

- padded current-step AABB filtering
- future motion-based conservative culling
- candidate rebuilds inside Newton or line search

## System Role

`CollisionCandidates` is the final output of the current contact preprocessing stage.

Future layers will consume it in two ways:

- barrier assembly will turn each candidate into a local PT or EE distance input
- CCD will evaluate the same candidate topology at trial states along a search direction

The important architectural point is that candidate generation remains topology-centric. It does not evaluate barrier energy and it does not decide final active contacts. It only defines which primitive pairs are worth passing down.
