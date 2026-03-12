# Rigid Body Dynamics Theory and Implementation

This document explains the rigid-body simulation implemented in the current `rtr::system::physics` runtime. The implementation described here corresponds to `PhysicsSystem -> RigidBodyWorld -> collision pair generation`, not to the older `PhysicsWorld` / leapfrog-based design.

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

**Semi-implicit Euler Integration Implementation:**
To balance simplicity and stability, the engine currently uses a semi-implicit Euler update inside `RigidBodyWorld::integrate_body`:

```cpp
const auto acc = rb.state().inverse_mass() * rb.state().forces.accumulated_force;
rb.state().translation.linear_velocity += acc * delta_seconds;
rb.state().translation.linear_velocity *= rb.linear_decay();
rb.state().translation.position += rb.state().translation.linear_velocity * delta_seconds;
```

The order matters: velocity is updated first, and the new velocity is then used to advance position. This is the standard semi-implicit Euler pattern.

**Common Force Computation:**
- **Gravity**: $f_{gravity} = M\mathbf{g}$. If `use_gravity()` is enabled, gravity is added to the force accumulator before integration.
- **Force Accumulation**: Custom forces added by gameplay or framework code flow into `state.forces.accumulated_force`, which is converted into linear acceleration through inverse mass.

## 3. Rotational Motion and Quaternions

The state of rotational motion includes the quaternion $q$ (representing orientation) and angular velocity $\omega$ (direction represents the rotation axis, magnitude represents the rotation rate) (see `RotationalState` in the code).

**Why choose Quaternions?**
Compared to Euler angles (which can cause gimbal lock and are difficult to differentiate) and rotation matrices (which have 9 elements but only 3 degrees of freedom, causing severe redundancy), a quaternion uses only four numbers to perfectly represent a 3D rotation. The code uses `pbpt::math::Quat` to perform highly efficient rotational update calculations.

**Core Physical Quantities of Rotational Dynamics:**
- **Torque ($\tau$)**: Equivalent to Force in translational motion. In the code, this is `state.forces.accumulated_torque`. The formula is the cross product of the world moment arm vector and the force at the application point: $\tau_i = (Rr_i) \times f_i$.
- **Inertia Tensor ($I$)**: Equivalent to Mass. Since the object is rotating, its world-coordinate inertia tensor must be updated in real-time based on the current rotation matrix $R$: $I^{-1} = R I_{ref}^{-1} R^T$. The rigid body stores its local inverse inertia tensor (`m_inverse_inertia_tensor_ref`).
  In the system, this is encapsulated in `RigidBodyWorld::inverse_inertia_tensor_world`:
  ```cpp
  const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
  return rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
  ```

## 4. Complete Rigid Body Update Loop (Simulation Implementation)

In `RigidBodyWorld::step`, the engine updates the overall state $s = \{v, x, \omega, q\}$ in the following order:

1. **Integrate Dynamic Bodies**
   For every awake dynamic rigid body, the world:
   - adds gravity when enabled,
   - integrates linear velocity from accumulated force,
   - applies linear decay,
   - advances position with the updated velocity,
   - computes world-space inverse inertia,
   - integrates angular velocity from accumulated torque,
   - applies angular decay,
   - advances orientation and renormalizes the quaternion.
   
2. **Build Contact Snapshot**
   The world enumerates collider pairs, skips self-pairs and static-static pairs, converts colliders into `WorldCollider` instances, and calls the corresponding `ContactPairTrait<...>::generate(...)` routines from `collision/`.

3. **Lift Geometric Contacts into Solver Contacts**
   A valid `ContactResult` is converted into rigid-body-specific `Contact` and then `SolverContact` records. This stage computes:
   - body ids and collider ids,
   - contact arms `r_a`, `r_b`,
   - world-space inverse inertia tensors,
   - contact normal and tangent,
   - effective mass along normal and tangent directions,
   - restitution bias,
   - frame-local accumulated impulses initialized to zero.

4. **Velocity Phase: Projected Gauss-Seidel**
   The solver iterates `velocity_iterations` times over the frame snapshot. For each contact:
   - solve the normal impulse,
   - accumulate and clamp it to stay non-negative,
   - apply the actual delta impulse to both bodies,
   - solve the tangential impulse,
   - clamp friction magnitude against `mu * normal_impulse_sum`.

5. **Position Phase: Penetration Correction**
   The solver then runs `position_iterations` rounds of positional correction using the contact normal and penetration depth stored in the snapshot. This does not rebuild contact geometry within the same frame.

6. **Clear External Forces**
   After solving, every rigid body clears its force and torque accumulators so the next frame starts from a clean externally-driven state.

## 5. Rotational Update Details

Orientation is advanced with the standard quaternion derivative form:

```cpp
const auto angular_velocity_quat = pbpt::math::Quat(
    0.0f,
    rb.state().rotation.angular_velocity.x(),
    rb.state().rotation.angular_velocity.y(),
    rb.state().rotation.angular_velocity.z());
rb.state().rotation.orientation +=
    0.5f * delta_seconds * angular_velocity_quat * rb.state().rotation.orientation;
rb.state().rotation.orientation = pbpt::math::normalize(rb.state().rotation.orientation);
```

This is not a half-step angular-velocity integrator in the current codebase; it is a direct semi-implicit update with normalization for numerical stability.

## 6. Collision / Solver Boundary

The rigid-body solver intentionally owns the coupling-side types:

- `rigid_body/collider.hpp` defines body-attached colliders with `RigidBodyID`.
- `rigid_body/contact.hpp` defines `Contact` and `SolverContact`.
- `collision/contact.hpp` only defines `ContactResult` and `ContactPairTrait`.

This keeps collision detection reusable while ensuring solver-facing state remains local to the rigid-body module.
