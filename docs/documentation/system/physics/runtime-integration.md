# Physics Runtime Integration

The authoritative version of this page is currently the Chinese document `runtime-integration.zh.md`.

This English page exists to keep the bilingual MkDocs structure aligned.

## Current Runtime Contract

`step_scene_physics(scene, physics_system, dt)` is the framework-level entry point:

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
sync_scene_to_cloth(scene, physics_system.cloth_world());
physics_system.step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());
sync_cloth_to_scene(scene, physics_system.cloth_world());
```

Key points:

- `PhysicsSystem::step()` performs simulation only.
- Scene synchronization lives in `src/rtr/framework/integration/physics/`.
- Cloth runtime state is owned by `ClothWorld` after registration.
- Dynamic rigid-body poses are owned by `RigidBodyWorld` during simulation.
