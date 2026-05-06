# Physics Runtime Integration

This page describes how the framework-level scene/runtime loop connects rigid-body physics, the IPC deformable system, and render-facing mesh components.

## Current Fixed-Step Entry Point

The authoritative runtime entry point is `step_scene_physics(scene, physics_system, dt)`:

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_system());
sync_scene_to_ipc(scene, physics_system.ipc_system());

physics_system.rigid_body_system().step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_system());

physics_system.ipc_system().step(dt);
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

Two details matter here:

- rigid-body stepping and IPC stepping are both explicit in the framework integration layer;
- scene-to-runtime sync happens before solving, runtime-to-scene sync happens after solving.

## Why the framework steps both systems explicitly

The rigid-body system uses the frame/fixed-tick `dt` pushed in from the runtime loop.

The IPC system also consumes the fixed-tick `dt` from the runtime loop and solves a nonlinear backward-Euler optimization step. Keeping both `rigid_body_system().step(dt)` and `ipc_system().step(dt)` explicit in `scene_physics_step(...)` makes the fixed-step boundary obvious and keeps the pre-sync / post-sync structure visible.

In the current demo/example path, `AppRuntimeConfig::fixed_delta_seconds` is often set to `0.01` so it also works well with the deformable solver settings.

## Scene / Physics / Renderer Ownership

| Layer | Stores | Runtime authority |
| --- | --- | --- |
| Scene Graph | game objects, hierarchy, component wiring | framework layer |
| `rb::RigidBodySystem` | rigid-body state, colliders, contacts | rigid-body runtime |
| `ipc::IPCSystem` / `ipc::IPCState` | deformable nodal state, masses, per-body offsets | IPC runtime |
| `DeformableMeshComponent` | render-facing surface copy for the GPU | renderer-facing cache |

The important split is:

- rigid bodies own dynamic transforms while simulating;
- IPC owns deformable nodal positions while simulating;
- `DeformableMeshComponent` is a render-facing cache, not the authoritative deformable state.

## Rigid-Body Direction

Rigid-body runtime integration still follows the usual pre-sync / solve / post-sync pattern:

```text
Scene -> sync_scene_to_rigid_body(...) -> rb::RigidBodySystem
rb::RigidBodySystem -> sync_rigid_body_to_scene(...) -> Scene
```

Dynamic rigid bodies write transforms back to the scene graph after simulation. Static and non-dynamic bodies are still driven from the scene side during the pre-pass.

## IPC Direction

IPC still uses a slightly different ownership pattern:

- scene-to-IPC is dirty-driven source synchronization, not per-frame scene-transform overwrite;
- IPC-to-scene happens every fixed tick after `ipc_system().step(dt)`.

That is because current `TetBody` instances are runtime-owned deformable bodies after registration. The scene graph does not continuously overwrite nodal positions.

The bridge objects are:

- `IPCTetComponent`
- `sync_ipc_to_scene(...)`
- `DeformableMeshComponent`

The actual registration/write-back details are documented in:

- [`ipc-scene-bridge.md`](ipc-scene-bridge.md)
- [`ipc-fixed-end-block-example.md`](ipc-fixed-end-block-example.md)

## Current Runtime Flow

```text
GameObject / Components
    |
    +--> RigidBody / Collider components
    |        |
    |        \--> sync_scene_to_rigid_body(...)
    |
    \--> DeformableMeshComponent + IPCTetComponent
             |
             \--> sync_ipc_to_scene(...)

rb::RigidBodySystem::step(dt)

ipc::IPCSystem::step(dt)
    -> update IPCState::x
    -> sync_ipc_to_scene(...)
    -> apply_deformed_surface(...)
```

The deformable path deliberately writes through `DeformableMeshComponent::apply_deformed_surface(...)`, so the CPU mesh stays normalized and the render resource can lazily upload updated vertices to the GPU.
