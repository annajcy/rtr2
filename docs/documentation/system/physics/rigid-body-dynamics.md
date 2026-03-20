# Rigid Body Dynamics Theory and Implementation

This page documents the rigid-body half of the current RTR2 physics runtime. The broader runtime now includes both `RigidBodyWorld` and `ClothWorld`; detailed architecture and algorithm notes are maintained in the Chinese pages under `docs/documentation/system/physics/*.zh.md`.

## Runtime Context

- Scene/physics synchronization is performed by `step_scene_physics(...)`.
- `PhysicsSystem::step(dt)` currently calls `RigidBodyWorld::step(dt)` first and `ClothWorld::step(dt)` second.
- This page focuses only on `RigidBodyWorld`.

## Current Pipeline

```text
for each rigid body:
    integrate_body(body, dt)

contacts = collect_contacts()
solver_contacts = build_solver_contacts(contacts)

repeat velocity iterations:
    apply_velocity_impulses(contact)

repeat position iterations:
    apply_position_correction(contact)

clear external forces
```

## Algorithms in Use

- Translational integration: semi-implicit Euler.
- Rotational integration: world-space inverse inertia update + quaternion derivative step.
- Contact generation: `collision/` pair traits generate geometric contacts.
- Velocity solve: projected Gauss-Seidel with accumulated normal/tangent impulses.
- Position solve: frame-local penetration correction without rebuilding contact geometry.

## Code Map

- `src/rtr/system/physics/rigid_body/rigid_body_world.hpp`
- `src/rtr/system/physics/rigid_body/contact.hpp`
- `src/rtr/system/physics/collision/*.hpp`
- `src/rtr/framework/integration/physics/rigid_body_scene_sync.hpp`
