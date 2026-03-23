# Physics System Overview

RTR2 currently has two physics directions under `src/rtr/system/physics/`:

- a runtime-connected rigid-body subsystem in `rigid_body/`
- a first runtime-connected IPC/FEM deformable path in `ipc/`

`PhysicsSystem` now owns both `rb::RigidBodySystem` and `ipc::IPCSystem`. The rigid-body runtime is still the more complete subsystem, but the IPC side already has its first scene bridge and editor example.

## Current Scope

- Implemented: rigid-body simulation, collision detection/response, scene/physics synchronization, IPC/FEM core data structures, IPC solver loop, IPC scene write-back, IPC fixed-end demo.
- Not implemented: cloth runtime, IPC contact/barrier/CCD, rigid-body/IPC coupling.
  Contact/barrier/CCD and rigid-body/IPC coupling are still future work.

## Fixed-Step Flow

```text
step_scene_physics(scene, physics_system, dt)
    -> sync_scene_to_rigid_body(...)
    -> rb::RigidBodySystem::step(dt)
    -> sync_rigid_body_to_scene(...)
    -> ipc_system.step(dt)
    -> sync_ipc_to_scene(...)
```

Scene synchronization stays in the framework integration layer, and both rigid-body and IPC stepping are now explicit parts of that fixed-step flow.

## Runtime Ownership

| Location | Stores | Runtime authority |
| --- | --- | --- |
| Scene Graph | game objects, hierarchy, render-facing components | framework layer |
| `rb::RigidBodySystem` | rigid-body state, colliders, contacts, solver state | rigid-body runtime |
| `ipc::IPCSystem` / `ipc::IPCState` | deformable global nodal state, masses, per-body offsets | deformable runtime |

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
- [`ipc-scene-bridge.md`](ipc-scene-bridge.md)
- [`ipc-fixed-end-block-example.md`](ipc-fixed-end-block-example.md)
- [`rigid-body-dynamics.md`](rigid-body-dynamics.md)

For the deformable data layer, continue with:

- [`ipc/overview.md`](ipc/overview.md)
