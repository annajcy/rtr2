# Physics System Overview

This section describes the custom physics engine (`rtr::system::physics`) used in RTR2. The physics engine primarily uses semi-implicit Euler integration (Leapfrog integration) for rigid body dynamics simulation.

## Architecture Overview

The physics module's architecture is divided into the following main layers and components:

1. **`scene_physics_sync.hpp` (Framework-to-Physics Adapter)**
   - **Responsibility**: Connects the independent physics world with upper-level framework components (`GameObject`, mainly `RigidBody` and `Collider` components).
   - **Interaction Flow**: The runtime performs `sync_scene_to_physics()` before stepping the world, then calls `PhysicsWorld::tick()`, and finally applies `sync_physics_to_scene()` so dynamic rigid body poses are written back to scene nodes.
2. **`PhysicsWorld` (Core Physics Sandbox)**
   - **Responsibility**: Maintains all rigid body structures (`RigidBody`) and collider structures (`Collider`), acting as the core object where physical simulation occurs.
   - **Core Pipeline (`tick`)**: In each frame's time update, it sequentially executes force and displacement integration (`integrate_forces_and_drift`), collision detection (`generate_contacts`), collision constraint solving (`solve_contacts`), and finally updates externally observable velocities (`update_observable_velocities`).
3. **`RigidBody` (Rigid Body Data Container)**
   - **Responsibility**: A pure data object containing the object's motion state (`RigidBodyState`), divided into translational state (`TranslationState`: position and velocity) and rotational state (`RotationalState`: quaternion orientation and angular velocity), along with a force accumulator (`ForceAccumulator`).
   - **Special Fields**: To support leapfrog semi-implicit integration, the class also maintains half-step velocities (`m_half_step_linear_velocity` and `m_half_step_angular_velocity`).
