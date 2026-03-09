# Physics System Overview

This section describes the custom physics engine (`rtr::system::physics`) used in RTR2. The physics engine primarily uses semi-implicit Euler integration (Leapfrog integration) for rigid body dynamics simulation.

## Architecture Overview

The physics module's architecture is divided into the following main layers and components:

1. **`PhysicsSystem` (Bridge between Physics and Scene)**
   - **Responsibility**: Connects the underlying independent physics world with the upper-level framework components (ECS-structured `GameObject`, mainly `RigidBody` and `Collider` components).
   - **Interaction Flow**: In the engine's fixed time step (`fixed_tick`), it sequentially calls `sync_scene_to_physics()` (syncs node poses to colliders and kinematic rigid bodies), `m_physics_world.tick()` for simulation stepping, and finally `sync_physics_to_scene()` to sync the latest poses of dynamic rigid bodies back to the scene nodes.
2. **`PhysicsWorld` (Core Physics Sandbox)**
   - **Responsibility**: Maintains all rigid body structures (`RigidBody`) and collider structures (`Collider`), acting as the core object where physical simulation occurs.
   - **Core Pipeline (`tick`)**: In each frame's time update, it sequentially executes force and displacement integration (`integrate_forces_and_drift`), collision detection (`generate_contacts`), collision constraint solving (`solve_contacts`), and finally updates externally observable velocities (`update_observable_velocities`).
3. **`RigidBody` (Rigid Body Data Container)**
   - **Responsibility**: A pure data object containing the object's motion state (`RigidBodyState`), divided into translational state (`TranslationState`: position and velocity) and rotational state (`RotationalState`: quaternion orientation and angular velocity), along with a force accumulator (`ForceAccumulator`).
   - **Special Fields**: To support leapfrog semi-implicit integration, the class also maintains half-step velocities (`m_half_step_linear_velocity` and `m_half_step_angular_velocity`).
