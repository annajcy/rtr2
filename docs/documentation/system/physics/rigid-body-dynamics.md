# Rigid Body Dynamics Theory and Implementation

This document provides a detailed breakdown of the rigid body dynamics theory and its corresponding implementation in the `rtr::system::physics` engine.

## 1. Introduction

**Core Objective:**
The core of physical engine simulation is to continuously update the state variable set $s^{[k]} = \{v, x, \omega, q\}$ of rigid bodies over time.

**Motion Decomposition and Equation Representation:**
Because rigid bodies do not deform, all of their complex motions in space can be perfectly divided into two parts: Translation and Rotation.
The world coordinate of any point on a rigid body is represented as $x + Rr_i$, meaning the local coordinate $r_i$ is first rotated by the rotation matrix $R$, and then added to the rigid body's center of mass translation $x$. In the engine, the object's `position` corresponds to the translation $x$, and its `orientation` (quaternion) corresponds to the rotation matrix $R$.

## 2. Translational Motion and Numerical Integration

The state of translational motion includes position $x$ and linear velocity $v$ (see `TranslationState` in the code).

**Comparison of Numerical Integration Methods:**
Updating position and velocity is essentially calculating the area under the velocity/acceleration curve:
- **Explicit Euler**: First-order accuracy, uses the velocity at the start of the time interval as the rectangle height. Truncation error is $O(\Delta t^2)$.
- **Implicit Euler**: First-order accuracy, uses the velocity at the end of the time interval. Truncation error is also $O(\Delta t^2)$.
- **Mid-point**: Second-order accuracy, uses the velocity at the midpoint of the time interval. The errors introduced by the first derivative can cancel each other out, reducing the final truncation error to $O(\Delta t^3)$.

**Leapfrog / Semi-implicit Integration Implementation:**
To achieve a balance between computational performance and stability, the physical engine practically uses the Leapfrog method (an application paradigm of the mid-point method). This is reflected in `PhysicsWorld::integrate_forces_and_drift`:

```cpp
// 1. Initialization and caching of half-step velocity (predicting v^{[-0.5]} -> v^{[0.5]})
if (!body.linear_half_step_initialized()) {
    body.initialize_half_step_linear_velocity(
        state.translation.linear_velocity + acceleration * (0.5f * delta_seconds));
}

// 2. Implicitly update position: x^{[1]} = x^{[0]} + \Delta t v^{[0.5]}
state.translation.position += body.half_step_linear_velocity() * delta_seconds;

// 3. Explicitly update half-step velocity: Accumulating v^{[0.5]}
body.half_step_linear_velocity() += acceleration * delta_seconds;
```
*(Note: Here, `half_step_linear_velocity` is effectively the midpoint velocity estimate achieved by shifting the integration by half a time step.)*

**Common Force Computation:**
- **Gravity**: $f_{gravity}^{[0]} = M\mathbf{g}$. In code, this is directly implemented as: `const pbpt::math::Vec3 gravity_force = body.use_gravity() ? (m_gravity * state.mass) : pbpt::math::Vec3(0.0f);`
- **Force Accumulation**: Custom forces (like drag) added by the user flow directly into `state.forces.accumulated_force`. Together, they compute the acceleration: `acceleration = total_force / state.mass`.

## 3. Rotational Motion and Quaternions

The state of rotational motion includes the quaternion $q$ (representing orientation) and angular velocity $\omega$ (direction represents the rotation axis, magnitude represents the rotation rate) (see `RotationalState` in the code).

**Why choose Quaternions?**
Compared to Euler angles (which can cause gimbal lock and are difficult to differentiate) and rotation matrices (which have 9 elements but only 3 degrees of freedom, causing severe redundancy), a quaternion uses only four numbers to perfectly represent a 3D rotation. The code uses `pbpt::math::Quat` to perform highly efficient rotational update calculations.

**Core Physical Quantities of Rotational Dynamics:**
- **Torque ($\tau$)**: Equivalent to Force in translational motion. In the code, this is `state.forces.accumulated_torque`. The formula is the cross product of the world moment arm vector and the force at the application point: $\tau_i = (Rr_i) \times f_i$.
- **Inertia Tensor ($I$)**: Equivalent to Mass. Since the object is rotating, its world-coordinate inertia tensor must be updated in real-time based on the current rotation matrix $R$: $I^{-1} = R I_{ref}^{-1} R^T$. The rigid body stores its local inverse inertia tensor (`m_inverse_inertia_tensor_ref`).
  In the system, this is encapsulated in `PhysicsWorld::inverse_inertia_tensor_world`:
  ```cpp
  const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
  return rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
  ```

## 4. Complete Rigid Body Update Loop (Simulation Implementation)

In the underlying `PhysicsWorld::tick`, the engine updates the overall state $s = \{v, x, \omega, q\}$ in the following order:

1. **Preprocessing and Force/Torque Accumulation (Force Integration & Drift)**
   Accumulates external forces to convert to translational acceleration, and calculates angular acceleration by converting torque via the local pseudo-inertia tensor (i.e., the $I^{[0]} \leftarrow R^{[0]}I_{ref}(R^{[0]})^T$ operation mentioned above).
   
2. **Semi-implicit Update of Translational State (Translational Update)**
   Following the leapfrog method described earlier, advances `half_step_linear_velocity` and uses it to update the position `position` for the next frame.

3. **Semi-implicit Update of Rotational State (Rotational Update)**
   Similar to the translational state, the code also uses a half-step angular velocity to integrate and update the rotation orientation quaternion:
   ```cpp
   // Update orientation quaternion (quaternion incremental derivative multiplication integration)
   state.rotation.orientation = integrate_orientation(state.rotation.orientation, body.half_step_angular_velocity(), delta_seconds);
   // Update half-step angular acceleration
   body.half_step_angular_velocity() += angular_acceleration * delta_seconds;
   ```
   Where `integrate_orientation` strictly implements the formula $q^{[1]} \leftarrow q^{[0]} + \left[0 \quad \frac{\Delta t}{2}\omega^{[1]}\right] \times q^{[0]}$:
   ```cpp
   const pbpt::math::Quat delta = (omega_quat * orientation) * (0.5f * delta_seconds);
   return pbpt::math::normalize(...); // Renormalize to counteract accumulated floating-point errors
   ```

4. **Collision Point Processing and Fine-tuning (Solve Contacts)**
   After integration is complete, objects may interpenetrate. The system calls `solve_contacts` to calculate the impulse at each collision point, directly modifying the relative half-step velocities of the rigid bodies to bounce them apart, and applying a displacement correction (Penetration Adjustment) to mitigate sustained penetration issues.

5. **Synchronization of Observable Velocities (Update Observable Velocities)**
   Executes `update_observable_velocities`. Because the physics world's integration step is shifted by half a time step, the linear velocity $v$ and angular velocity $\omega$ are compensated back to align with the current time step so that the game logic layer can observe velocities that match the current positions.
