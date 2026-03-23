# Physics Runtime Integration

This page describes how the framework-level scene/runtime loop connects rigid-body physics, the IPC deformable system, and render-facing mesh components.

## Current Fixed-Step Entry Point

The authoritative runtime entry point is `step_scene_physics(scene, physics_system, dt)`:

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_system());
physics_system.step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_system());

physics_system.ipc_system().step();
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

Two details matter here:

- `PhysicsSystem::step(dt)` currently advances `rb::RigidBodySystem` only.
- IPC stepping and IPC-to-scene write-back are performed explicitly in the framework integration layer after rigid-body synchronization.

## Why IPC Is Stepped Outside `PhysicsSystem::step()`

The rigid-body system uses the frame/fixed-tick `dt` pushed in from the runtime loop.

The IPC system, by contrast, owns its own `IPCConfig::dt` and solves a nonlinear backward-Euler optimization step. Keeping `ipc_system().step()` outside `PhysicsSystem::step()` makes the fixed-step boundary explicit and keeps scene synchronization around the deformable write-back path.

In the current demo/example path, `AppRuntimeConfig::fixed_delta_seconds` is set to `0.01` so it matches the default `IPCConfig::dt`.

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

IPC currently uses a slightly different ownership pattern:

- scene-to-IPC happens at registration time, not every frame;
- IPC-to-scene happens every fixed tick after `ipc_system().step()`.

That is because current `TetBody` instances are registered once and then simulated as runtime-owned deformable bodies. The scene graph does not continuously overwrite nodal positions.

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

rb::RigidBodySystem <----- PhysicsSystem::step(dt)

ipc::IPCSystem::step()
    -> update IPCState::x
    -> sync_ipc_to_scene(...)
    -> apply_deformed_surface(...)
```

The deformable path deliberately writes through `DeformableMeshComponent::apply_deformed_surface(...)`, so the CPU mesh stays normalized and the render resource can lazily upload updated vertices to the GPU.
