# Physics System Overview

RTR2 now has a mixed physics runtime. `PhysicsSystem` owns both `RigidBodyWorld` and `ClothWorld`, while the framework layer drives synchronization through `step_scene_physics(scene, physics_system, dt)`.

The authoritative detailed write-up is currently the Chinese documentation:

- `overview.zh.md`
- `runtime-integration.zh.md`
- `cloth-simulation.zh.md`
- `rigid-body-dynamics.zh.md`

## Current Scope

- Implemented: rigid-body simulation, collision detection/response, mass-spring cloth, scene/physics synchronization, deformable mesh write-back.
- Not implemented: cloth collision, self-collision, cloth-rigid coupling, implicit cloth, PBD/XPBD cloth, FEM/water runtime.

## Fixed-Step Flow

```text
step_scene_physics(scene, physics_system, dt)
    -> sync_scene_to_rigid_body(...)
    -> sync_scene_to_cloth(...)
    -> PhysicsSystem::step(dt)
         -> RigidBodyWorld::step(dt)
         -> ClothWorld::step(dt)
    -> sync_rigid_body_to_scene(...)
    -> sync_cloth_to_scene(...)
```

`PhysicsSystem::step()` performs world-local simulation only. Scene synchronization stays in the framework integration layer.

## Algorithm Map

| Subsystem | Current algorithm | Primary entry points |
| --- | --- | --- |
| Rigid Body | semi-implicit Euler + contact generation + PGS + positional correction | `RigidBodyWorld::step()` |
| Cloth | explicit mass-spring + semi-implicit Euler + substeps + damping | `build_cloth_spring_network()`, `ClothWorld::step()` |
| Integration | scene/physics synchronization across fixed ticks | `step_scene_physics()` |
