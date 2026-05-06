# IPC Contact Overview

`src/rtr/system/physics/ipc/contact/` is the bridge layer between solver-facing global state and local contact primitives.

This layer exists because neither side below is sufficient on its own:

- `core/` knows about global DOFs and body registration, but not surface primitives
- `geometry/` knows how to evaluate one primitive pair locally, but not where those coordinates come from

`contact/` fills that gap by building a unified collision surface and enumerating candidate primitive pairs on top of it.

## Current Modules

The current directory contains:

- `collision_mesh.hpp`: unified surface vertices / edges / triangles built from deformable tet boundaries and static obstacle meshes
- `collision_candidates.hpp`: brute-force Day 2 broad phase with deterministic PT / EE candidate generation and filtering

## Responsibility Boundary

This layer is intentionally narrow:

- it owns surface topology and primitive ownership
- it owns the rule for reading a surface vertex position
- it owns broad-phase candidate enumeration
- it does not own barrier energy, CCD root finding, or sparse global assembly

That split keeps the stack clean:

```text
IPCSystem + ObstacleBody
    -> build_collision_mesh(...)
    -> unified surface primitive index space
    -> build_collision_candidates(...)
    -> PT / EE candidate lists
    -> future barrier / CCD wrappers
    -> geometry distance kernels
```

## Why Collision Mesh Exists

The global `IPCState` only stores deformable vertices. Static obstacle vertices do not live in the optimizer state at all.

Contact code still needs a single surface view that can answer:

- which body owns this surface primitive
- whether a vertex is deformable or static
- how to fetch the current world-space position
- how to map deformable vertices back to global DOFs later

`CollisionMesh` is that view. It is not a copy of the solver state. It is a topology-and-mapping layer.

## Day 2 Scope

The current implementation targets the first useful IPC contact slice:

- deformable tet bodies from `IPCSystem`
- static obstacle triangle meshes from `ObstacleBody`
- point-triangle candidates in both directions
- edge-edge candidates across deformable and obstacle sides

It does not yet handle:

- self-contact
- deformable-vs-deformable contact
- obstacle-vs-obstacle contact
- BVH or spatial-hash acceleration

## Architectural Role

`contact/` sits directly above `model/` and `core/`, and directly below future barrier / CCD logic.

- `model/mesh_tet_converter/tet_to_mesh.hpp` provides `TetSurfaceMapping`
- `core/ipc_system.hpp` provides body lists and deformable state
- `model/obstacle_body.hpp` provides static triangle meshes
- `contact/` merges those sources into one primitive index space
- future barrier / CCD code will consume candidates and build `geometry::Distance` inputs from them

That separation is the reason the geometry kernels can stay purely local while the higher layers still work with a mixed deformable/static scene.
