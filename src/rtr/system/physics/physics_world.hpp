#pragma once

#include <cmath>
#include <unordered_map>

#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/rigid_body.hpp"

namespace rtr::system::physics {

class PhysicsWorld {
private:
    std::unordered_map<RigidBodyID, RigidBody> m_rigid_bodies;
    RigidBodyID                              m_next_rigid_body_id{0};
    pbpt::math::Vec3                         m_gravity{0.0f, -9.81f, 0.0f};

    static bool is_valid_mass(pbpt::math::Float mass) {
        return std::isfinite(mass) && mass > 0.0f;
    }

    static bool is_finite_matrix(const pbpt::math::Mat3& matrix) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (!std::isfinite(matrix[row][col])) {
                    return false;
                }
            }
        }
        return true;
    }

    static pbpt::math::Quat integrate_orientation(const pbpt::math::Quat& orientation,
                                                  const pbpt::math::Vec3& angular_velocity,
                                                  float delta_seconds) {
        const pbpt::math::Quat omega_quat(0.0f, angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        const pbpt::math::Quat delta = (omega_quat * orientation) * (0.5f * delta_seconds);
        return pbpt::math::normalize(pbpt::math::Quat(orientation.w() + delta.w(), orientation.x() + delta.x(),
                                                      orientation.y() + delta.y(), orientation.z() + delta.z()));
    }

public:
    RigidBodyID create_rigid_body(RigidBody rigid_body = RigidBody{}) {
        const RigidBodyID id = m_next_rigid_body_id++;
        m_rigid_bodies[id] = std::move(rigid_body);
        return id;
    }

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id); }
    bool remove_rigid_body(RigidBodyID id) { return m_rigid_bodies.erase(id) > 0; }

    RigidBody& get_rigid_body(RigidBodyID id) { return m_rigid_bodies.at(id); }
    const RigidBody& get_rigid_body(RigidBodyID id) const { return m_rigid_bodies.at(id); }

    const pbpt::math::Vec3& gravity() const { return m_gravity; }
    void set_gravity(const pbpt::math::Vec3& gravity) { m_gravity = gravity; }

    void tick(float delta_seconds) {
        for (auto& [id, body] : m_rigid_bodies) {
            (void)id;
            auto& state = body.state();
            if (body.type() != RigidBodyType::Dynamic || !body.is_awake() || !is_valid_mass(state.mass)) {
                body.clear_forces();
                continue;
            }

            // Apply gravity and accumulated forces to compute acceleration
            const pbpt::math::Vec3 gravity_force =
                body.use_gravity() ? (m_gravity * state.mass) : pbpt::math::Vec3(0.0f);
            const pbpt::math::Vec3 total_force  = state.forces.accumulated_force + gravity_force;
            const pbpt::math::Vec3 acceleration = total_force / state.mass;

            // Semi-implicit Euler integration with half-step velocity for improved stability
            if (!body.linear_half_step_initialized()) {
                body.initialize_half_step_linear_velocity(
                    state.translation.linear_velocity + acceleration * (0.5f * delta_seconds));
            }

            // use half-step velocity to update position, then update velocity with full acceleration   
            state.translation.position += body.half_step_linear_velocity() * delta_seconds;

            // Update velocity for the next frame
            body.half_step_linear_velocity() += acceleration * delta_seconds;
            state.translation.linear_velocity = body.half_step_linear_velocity() - acceleration * (0.5f * delta_seconds);

            if (is_finite_matrix(body.inverse_inertia_tensor_ref())) {
                const pbpt::math::Mat3 rotation_matrix = state.rotation.orientation.to_mat3();
                const pbpt::math::Mat3 inverse_inertia_tensor_world =
                    rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
                const pbpt::math::Vec3 angular_acceleration =
                    inverse_inertia_tensor_world * state.forces.accumulated_torque;

                if (!body.angular_half_step_initialized()) {
                    body.initialize_half_step_angular_velocity(
                        state.rotation.angular_velocity + angular_acceleration * (0.5f * delta_seconds));
                }

                state.rotation.orientation =
                    integrate_orientation(state.rotation.orientation, body.half_step_angular_velocity(), delta_seconds);
                body.half_step_angular_velocity() += angular_acceleration * delta_seconds;
                state.rotation.angular_velocity =
                    body.half_step_angular_velocity() - angular_acceleration * (0.5f * delta_seconds);
            }

            body.clear_forces();
        }
    }
};

}  // namespace rtr::system::physics
