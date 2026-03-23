#pragma once

#include <cmath>
#include <string>
#include <stdexcept>
#include <utility>

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::component {

class RigidBody final : public Component {
private:
    system::physics::rb::RigidBodySystem& m_physics_system;
    system::physics::rb::RigidBodyID      m_rigid_body_id{system::physics::rb::kInvalidRigidBodyId};
    bool                                  m_registered{false};
    system::physics::rb::RigidBody        m_source_body{};

    bool has_registered_body() const { return m_registered && m_physics_system.has_rigid_body(m_rigid_body_id); }

    static pbpt::math::Float sanitize_mass(pbpt::math::Float mass) {
        if (!std::isfinite(mass) || mass <= 0.0f) {
            throw std::invalid_argument("RigidBody mass must be finite and positive.");
        }
        return mass;
    }

    static pbpt::math::Mat3 sanitize_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (!std::isfinite(inverse_inertia_tensor_ref[row][col])) {
                    throw std::invalid_argument("RigidBody inverse inertia tensor must be finite.");
                }
            }
        }
        return inverse_inertia_tensor_ref;
    }

    system::physics::rb::RigidBody make_runtime_body_from_source() const {
        auto runtime_body = m_source_body;
        runtime_body.set_awake(true);
        runtime_body.state().translation.position = owner().node().world_position();
        runtime_body.state().rotation.orientation = owner().node().world_rotation();
        runtime_body.state().scale                = owner().node().world_scale();
        runtime_body.clear_forces();
        return runtime_body;
    }

    static pbpt::math::Float sanitize_restitution(pbpt::math::Float restitution) {
        if (!std::isfinite(restitution) || restitution < 0.0f || restitution > 1.0f) {
            throw std::invalid_argument("RigidBody restitution must be finite and within [0, 1].");
        }
        return restitution;
    }

    static pbpt::math::Float sanitize_friction(pbpt::math::Float friction) {
        if (!std::isfinite(friction) || friction < 0.0f) {
            throw std::invalid_argument("RigidBody friction must be finite and non-negative.");
        }
        return friction;
    }

    static pbpt::math::Float sanitize_decay(pbpt::math::Float decay, const char* label) {
        if (!std::isfinite(decay) || decay < 0.0f || decay > 1.0f) {
            throw std::invalid_argument(std::string(label) + " must be finite and within [0, 1].");
        }
        return decay;
    }

    system::physics::rb::RigidBody* physics_body() {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_physics_system.get_rigid_body(m_rigid_body_id);
    }

    const system::physics::rb::RigidBody* physics_body() const {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_physics_system.get_rigid_body(m_rigid_body_id);
    }

public:
    explicit RigidBody(core::GameObject& owner, system::physics::rb::RigidBodySystem& world, pbpt::math::Float mass = 1.0f,
                       system::physics::rb::RigidBodyType type = system::physics::rb::RigidBodyType::Dynamic,
                       bool use_gravity = true,
                       const pbpt::math::Mat3& inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros(),
                       pbpt::math::Float restitution = 0.0f,
                       pbpt::math::Float friction = 0.0f,
                       pbpt::math::Float linear_decay = 1.0f,
                       pbpt::math::Float angular_decay = 1.0f)
        : Component(owner),
          m_physics_system(world) {
        m_source_body.set_type(type);
        m_source_body.set_awake(true);
        m_source_body.set_use_gravity(use_gravity);
        m_source_body.set_restitution(sanitize_restitution(restitution));
        m_source_body.set_friction(sanitize_friction(friction));
        m_source_body.set_linear_decay(sanitize_decay(linear_decay, "RigidBody linear_decay"));
        m_source_body.set_angular_decay(sanitize_decay(angular_decay, "RigidBody angular_decay"));
        m_source_body.set_inverse_inertia_tensor_ref(
            sanitize_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref)
        );
        m_source_body.state().mass = sanitize_mass(mass);
    }

    void on_awake() override {}

    void on_enable() override {
        if (m_registered) {
            return;
        }

        auto runtime_body = make_runtime_body_from_source();
        m_rigid_body_id = m_physics_system.create_rigid_body(std::move(runtime_body));
        m_registered    = true;
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_system.remove_rigid_body(m_rigid_body_id);
        m_rigid_body_id = system::physics::rb::kInvalidRigidBodyId;
        m_registered    = false;
    }

    void on_destroy() override {}

    bool has_rigid_body() const { return has_registered_body(); }

    system::physics::rb::RigidBodyID rigid_body_id() const { return m_rigid_body_id; }
    const system::physics::rb::RigidBody& source_body() const { return m_source_body; }
    system::physics::rb::RigidBody* runtime_body() { return physics_body(); }
    const system::physics::rb::RigidBody* runtime_body() const { return physics_body(); }

    system::physics::rb::RigidBodyType type() const {
        const auto* body = physics_body();
        return body != nullptr ? body->type() : m_source_body.type();
    }

    void set_type(system::physics::rb::RigidBodyType type) {
        m_source_body.set_type(type);
        if (auto* body = physics_body(); body != nullptr) {
            body->set_type(type);
        }
    }

    pbpt::math::Vec3 position() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return owner().node().world_position();
        }
        return body->state().translation.position;
    }

    void set_position(const pbpt::math::Vec3& position) {
        owner().node().set_world_position(position);
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().translation.position = position;
    }

    pbpt::math::Vec3 linear_velocity() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return m_source_body.state().translation.linear_velocity;
        }
        return body->state().translation.linear_velocity;
    }

    void set_linear_velocity(const pbpt::math::Vec3& linear_velocity) {
        m_source_body.state().translation.linear_velocity = linear_velocity;
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().translation.linear_velocity = linear_velocity;
    }

    pbpt::math::Quat orientation() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return owner().node().world_rotation();
        }
        return body->state().rotation.orientation;
    }

    void set_orientation(const pbpt::math::Quat& orientation) {
        const auto normalized = pbpt::math::normalize(orientation);
        owner().node().set_world_rotation(normalized);
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().rotation.orientation = normalized;
    }

    pbpt::math::Vec3 angular_velocity() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return m_source_body.state().rotation.angular_velocity;
        }
        return body->state().rotation.angular_velocity;
    }

    void set_angular_velocity(const pbpt::math::Vec3& angular_velocity) {
        m_source_body.state().rotation.angular_velocity = angular_velocity;
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().rotation.angular_velocity = angular_velocity;
    }

    pbpt::math::Float mass() const {
        const auto* body = physics_body();
        return body != nullptr ? body->state().mass : m_source_body.state().mass;
    }

    void set_mass(pbpt::math::Float mass) {
        m_source_body.state().mass = sanitize_mass(mass);
        if (auto* body = physics_body(); body != nullptr) {
            body->state().mass = m_source_body.state().mass;
        }
    }

    bool use_gravity() const {
        const auto* body = physics_body();
        return body != nullptr ? body->use_gravity() : m_source_body.use_gravity();
    }

    void set_use_gravity(bool use_gravity) {
        m_source_body.set_use_gravity(use_gravity);
        if (auto* body = physics_body(); body != nullptr) {
            body->set_use_gravity(use_gravity);
        }
    }

    pbpt::math::Float restitution() const {
        const auto* body = physics_body();
        return body != nullptr ? body->restitution() : m_source_body.restitution();
    }

    void set_restitution(pbpt::math::Float restitution) {
        m_source_body.set_restitution(sanitize_restitution(restitution));
        if (auto* body = physics_body(); body != nullptr) {
            body->set_restitution(m_source_body.restitution());
        }
    }

    pbpt::math::Float friction() const {
        const auto* body = physics_body();
        return body != nullptr ? body->friction() : m_source_body.friction();
    }

    void set_friction(pbpt::math::Float friction) {
        m_source_body.set_friction(sanitize_friction(friction));
        if (auto* body = physics_body(); body != nullptr) {
            body->set_friction(m_source_body.friction());
        }
    }

    pbpt::math::Float linear_decay() const {
        const auto* body = physics_body();
        return body != nullptr ? body->linear_decay() : m_source_body.linear_decay();
    }

    void set_linear_decay(pbpt::math::Float linear_decay) {
        m_source_body.set_linear_decay(sanitize_decay(linear_decay, "RigidBody linear_decay"));
        if (auto* body = physics_body(); body != nullptr) {
            body->set_linear_decay(m_source_body.linear_decay());
        }
    }

    pbpt::math::Float angular_decay() const {
        const auto* body = physics_body();
        return body != nullptr ? body->angular_decay() : m_source_body.angular_decay();
    }

    void set_angular_decay(pbpt::math::Float angular_decay) {
        m_source_body.set_angular_decay(sanitize_decay(angular_decay, "RigidBody angular_decay"));
        if (auto* body = physics_body(); body != nullptr) {
            body->set_angular_decay(m_source_body.angular_decay());
        }
    }

    pbpt::math::Mat3 inverse_inertia_tensor_ref() const {
        const auto* body = physics_body();
        return body != nullptr ? body->inverse_inertia_tensor_ref() : m_source_body.inverse_inertia_tensor_ref();
    }

    void set_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        m_source_body.set_inverse_inertia_tensor_ref(
            sanitize_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref)
        );
        if (auto* body = physics_body(); body != nullptr) {
            body->set_inverse_inertia_tensor_ref(m_source_body.inverse_inertia_tensor_ref());
        }
    }

    void add_force(const pbpt::math::Vec3& force) {
        if (auto* body = physics_body(); body != nullptr) {
            body->state().forces.accumulated_force += force;
        }
    }

    void add_torque(const pbpt::math::Vec3& torque) {
        if (auto* body = physics_body(); body != nullptr) {
            body->state().forces.accumulated_torque += torque;
        }
    }

    void add_force_at_point(const pbpt::math::Vec3& force, const pbpt::math::Vec3& world_point) {
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().forces.accumulated_force += force;
        const pbpt::math::Vec3 lever_arm = world_point - body->state().translation.position;
        body->state().forces.accumulated_torque += pbpt::math::cross(lever_arm, force);
    }

    void clear_forces() {
        if (auto* body = physics_body(); body != nullptr) {
            body->clear_forces();
        }
    }

    void reset_dynamics() {
        if (auto* body = physics_body(); body != nullptr) {
            body->reset_dynamics();
        }
    }

    void reset_translation_dynamics() {
        if (auto* body = physics_body(); body != nullptr) {
            body->reset_translation_dynamics();
        }
    }

    void reset_rotational_dynamics() {
        if (auto* body = physics_body(); body != nullptr) {
            body->reset_rotational_dynamics();
        }
    }
};

}  // namespace rtr::framework::component
