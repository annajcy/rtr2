# Physics System Overview

This section describes the current `rtr::system::physics` architecture in RTR2. At this stage the physics system is a rigid-body-only runtime: `PhysicsSystem` orchestrates scene synchronization and delegates simulation to `RigidBodyWorld`, while the collision module provides shared geometric contact generation utilities.

## Architecture Overview

The physics module is currently organized into the following layers:

1. **`PhysicsSystem` (Runtime-Level Orchestrator)**
   - **Responsibility**: Owns the active physics runtime used by `AppRuntime`.
   - **Interaction Flow**: `PhysicsSystem::step(scene, dt)` first synchronizes framework-side transforms and collider state into the rigid-body world, then calls `RigidBodyWorld::step(dt)`, and finally writes dynamic rigid-body poses back into the scene graph.
2. **`rigid_body_scene_sync.hpp` (Framework-to-Rigid-Body Adapter)**
   - **Responsibility**: Bridges `GameObject` components and the rigid-body simulation state.
   - **Inbound Sync**: Pushes scene transforms, scale, and collider configuration into `RigidBodyWorld`.
   - **Outbound Sync**: Pulls dynamic rigid-body poses from `RigidBodyWorld` back into scene nodes after the fixed-step simulation finishes.
3. **`RigidBodyWorld` (Core Simulation World)**
   - **Responsibility**: Maintains all rigid bodies and body-attached colliders, integrates motion, collects contacts, and solves collision response.
   - **Core Pipeline (`step`)**: In each fixed step, the world integrates dynamic bodies, builds one frame-local contact snapshot, runs velocity-phase projected Gauss-Seidel iterations, runs positional penetration correction iterations, and then clears external force accumulators.
   - **Scope**: This world only manages rigid-body state. There is currently no cloth/FEM/water runtime in the active physics pipeline.
4. **`RigidBody` (Rigid Body Data Container)**
   - **Responsibility**: Stores the motion state (`RigidBodyState`), material-like response parameters (friction, restitution), sleep/awake state, mass properties, and force/torque accumulators.
4. **`collision/` (Geometric Contact Layer)**
   - **Responsibility**: Provides pure collision-detection data structures and pairwise contact generation.
   - **Contained Concepts**: `ColliderShape`, `WorldCollider`, `ContactResult`, and `ContactPairTrait<...>::generate(...)`.
   - **Boundary**: This layer does not own rigid-body identities or solver-side contact caches. Those coupling types live in `rigid_body/`.

## Current Directory Layout

```text
src/rtr/system/physics/
  physics_system.hpp
  common/
    physics_ids.hpp
    physics_material.hpp
    physics_step_context.hpp
  collision/
    collider_shape.hpp
    contact.hpp
    sphere_sphere.hpp
    sphere_box.hpp
    sphere_plane.hpp
    box_box.hpp
    box_plane.hpp
    mesh_plane.hpp
    plane_common.hpp
  rigid_body/
    rigid_body.hpp
    rigid_body_type.hpp
    collider.hpp
    contact.hpp
    rigid_body_world.hpp
```

## Fixed-Step Runtime Contract

At runtime, the fixed-step path is:

1. `AppRuntime` accumulates elapsed time and executes fixed updates.
2. `PhysicsSystem::step(scene, fixed_dt)` is called for each fixed tick.
3. `sync_scene_to_rigid_body(scene, rigid_body_world)` updates rigid-body and collider inputs.
4. `RigidBodyWorld::step(fixed_dt)` runs integration plus collision response.
5. `sync_rigid_body_to_scene(scene, rigid_body_world)` writes dynamic rigid-body transforms back.

This means the authoritative simulation state for dynamic rigid bodies lives in `RigidBodyWorld`, while the scene graph remains the render-facing representation.

## Collision / Rigid-Body Boundary

The current boundary is intentionally split as follows:

- `collision/` owns pure geometry and contact generation.
- `rigid_body/` owns `Collider`, `Contact`, `SolverContact`, and the impulse solver.

That split keeps pairwise collision routines reusable while keeping rigid-body identities, accumulated impulses, and solver-specific state inside the rigid-body module where they are consumed.
