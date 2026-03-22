# Physics System Overview

RTR2 currently has two physics directions under `src/rtr/system/physics/`:

- a runtime-connected rigid-body subsystem in `rigid_body/`
- an in-progress IPC/FEM subsystem in `ipc/`

`PhysicsSystem` currently owns only `RigidBodyWorld`. The IPC subtree already contains its first batch of data structures, but it is not wired into the runtime loop yet.

## Current Scope

- Implemented: rigid-body simulation, collision detection/response, scene/physics synchronization, IPC/FEM core data structures.
- Not implemented: cloth runtime, IPC solver loop, IPC contact/barrier/CCD, rigid-body/IPC coupling.

## Fixed-Step Flow

```text
step_scene_physics(scene, physics_system, dt)
    -> sync_scene_to_rigid_body(...)
    -> PhysicsSystem::step(dt)
         -> RigidBodyWorld::step(dt)
    -> sync_rigid_body_to_scene(...)
```

`PhysicsSystem::step()` performs world-local simulation only. Scene synchronization stays in the framework integration layer. The IPC subsystem is not part of this flow yet.

## Runtime Ownership

| Location | Stores | Runtime authority |
| --- | --- | --- |
| Scene Graph | game objects, hierarchy, render-facing components | framework layer |
| `RigidBodyWorld` | rigid-body state, colliders, contacts, solver state | rigid-body runtime |
| `ipc::IPCState` | future deformable global nodal state | future deformable runtime |

Once dynamic rigid bodies start simulating, scene transforms should no longer be treated as the authoritative dynamic state. The same ownership split is intended for deformables once the IPC runtime loop exists.

## IPC Documentation Subtree

Detailed IPC documentation now lives in the mirrored subtree under [`documentation/system/physics/ipc/`](ipc/overview.md).

- [`ipc/overview.md`](ipc/overview.md): directory responsibilities and capability boundary
- [`ipc/core/ipc_state.md`](ipc/core/ipc_state.md): global `3N` state layout and equations
- [`ipc/model/ipc_body.md`](ipc/model/ipc_body.md): body categories and global-DOF mapping metadata
- [`ipc/model/tet_body.md`](ipc/model/tet_body.md): `TetGeometry`, `TetBody`, rest-shape precompute, and block generators
- [`ipc/model/tet_mesh_convert.md`](ipc/model/tet_mesh_convert.md): tet-surface extraction and tet-to-render-mesh write-back
- [`ipc/model/obstacle_body.md`](ipc/model/obstacle_body.md): current obstacle placeholder semantics

That subtree follows the source layout: each `ipc/*.hpp` has its own page, and each directory has its own `overview.md`.

## Reading Order

For runtime-connected physics, continue with:

- [`runtime-integration.md`](runtime-integration.md)
- [`rigid-body-dynamics.md`](rigid-body-dynamics.md)

For the deformable data layer, continue with:

- [`ipc/overview.md`](ipc/overview.md)
