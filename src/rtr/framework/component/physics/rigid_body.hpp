#pragma once

#include <cmath>
#include <stdexcept>
#include <utility>

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::component {

class RigidBody final : public Component {
private:
    system::physics::PhysicsWorld& m_physics_world;
    system::physics::RigidBodyID   m_rigid_body_id{};
    system::physics::RigidBodyType m_type{system::physics::RigidBodyType::Dynamic};
    pbpt::math::Float              m_mass{1.0f};
    bool                           m_use_gravity{true};
    pbpt::math::Float              m_restitution{0.0f};
    pbpt::math::Float              m_friction{0.0f};
    pbpt::math::Mat3               m_inverse_inertia_tensor_ref{pbpt::math::Mat3::zeros()};
    bool                           m_registered{false};

    bool has_registered_body() const { return m_registered && m_physics_world.has_rigid_body(m_rigid_body_id); }

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

    system::physics::RigidBody* physics_body() {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_physics_world.get_rigid_body(m_rigid_body_id);
    }

    const system::physics::RigidBody* physics_body() const {
        if (!has_registered_body()) {
            return nullptr;
        }
        return &m_physics_world.get_rigid_body(m_rigid_body_id);
    }

public:
    explicit RigidBody(core::GameObject& owner, system::physics::PhysicsWorld& world, pbpt::math::Float mass = 1.0f,
                       system::physics::RigidBodyType type = system::physics::RigidBodyType::Dynamic,
                       bool use_gravity = true,
                       const pbpt::math::Mat3& inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros(),
                       pbpt::math::Float restitution = 0.0f,
                       pbpt::math::Float friction = 0.0f)
        : Component(owner),
          m_physics_world(world),
          m_type(type),
          m_mass(sanitize_mass(mass)),
          m_use_gravity(use_gravity),
          m_restitution(sanitize_restitution(restitution)),
          m_friction(sanitize_friction(friction)),
          m_inverse_inertia_tensor_ref(sanitize_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref)) {}

    void on_awake() override {}

    void on_enable() override {
        if (m_registered) {
            return;
        }

        system::physics::RigidBody body;
        body.set_type(m_type);
        body.set_awake(true);
        body.set_use_gravity(m_use_gravity);
        body.set_restitution(m_restitution);
        body.set_friction(m_friction);
        body.set_inverse_inertia_tensor_ref(m_inverse_inertia_tensor_ref);
        body.state().mass                        = m_mass;
        body.state().translation.position        = owner().node().world_position();
        body.state().translation.linear_velocity = pbpt::math::Vec3(0.0f);
        body.state().rotation.orientation        = owner().node().world_rotation();
        body.state().rotation.angular_velocity   = pbpt::math::Vec3(0.0f);
        body.state().scale                       = owner().node().world_scale();
        body.clear_forces();

        m_rigid_body_id = m_physics_world.create_rigid_body(std::move(body));
        m_registered    = true;
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_world.remove_rigid_body(m_rigid_body_id);
        m_rigid_body_id = system::physics::RigidBodyID{};
        m_registered    = false;
    }

    void on_destroy() override {}

    bool has_rigid_body() const { return has_registered_body(); }

    system::physics::RigidBodyID rigid_body_id() const { return m_rigid_body_id; }

    system::physics::RigidBodyType type() const {
        const auto* body = physics_body();
        return body != nullptr ? body->type() : m_type;
    }

    void set_type(system::physics::RigidBodyType type) {
        m_type = type;
        if (auto* body = physics_body(); body != nullptr) {
            body->set_type(type);
        }
    }

    pbpt::math::Vec3 position() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return pbpt::math::Vec3(0.0f);
        }
        return body->state().translation.position;
    }

    void set_position(const pbpt::math::Vec3& position) {
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().translation.position = position;
    }

    pbpt::math::Vec3 linear_velocity() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return pbpt::math::Vec3(0.0f);
        }
        return body->state().translation.linear_velocity;
    }

    pbpt::math::Quat orientation() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return pbpt::math::Quat::identity();
        }
        return body->state().rotation.orientation;
    }

    void set_orientation(const pbpt::math::Quat& orientation) {
        auto* body = physics_body();
        if (body == nullptr) {
            return;
        }
        body->state().rotation.orientation = pbpt::math::normalize(orientation);
    }

    pbpt::math::Vec3 angular_velocity() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return pbpt::math::Vec3(0.0f);
        }
        return body->state().rotation.angular_velocity;
    }

    pbpt::math::Float mass() const {
        const auto* body = physics_body();
        return body != nullptr ? body->state().mass : m_mass;
    }

    void set_mass(pbpt::math::Float mass) {
        m_mass = sanitize_mass(mass);
        if (auto* body = physics_body(); body != nullptr) {
            body->state().mass = m_mass;
        }
    }

    bool use_gravity() const {
        const auto* body = physics_body();
        return body != nullptr ? body->use_gravity() : m_use_gravity;
    }

    void set_use_gravity(bool use_gravity) {
        m_use_gravity = use_gravity;
        if (auto* body = physics_body(); body != nullptr) {
            body->set_use_gravity(use_gravity);
        }
    }

    pbpt::math::Float restitution() const {
        const auto* body = physics_body();
        return body != nullptr ? body->restitution() : m_restitution;
    }

    void set_restitution(pbpt::math::Float restitution) {
        m_restitution = sanitize_restitution(restitution);
        if (auto* body = physics_body(); body != nullptr) {
            body->set_restitution(m_restitution);
        }
    }

    pbpt::math::Float friction() const {
        const auto* body = physics_body();
        return body != nullptr ? body->friction() : m_friction;
    }

    void set_friction(pbpt::math::Float friction) {
        m_friction = sanitize_friction(friction);
        if (auto* body = physics_body(); body != nullptr) {
            body->set_friction(m_friction);
        }
    }

    pbpt::math::Mat3 inverse_inertia_tensor_ref() const {
        const auto* body = physics_body();
        return body != nullptr ? body->inverse_inertia_tensor_ref() : m_inverse_inertia_tensor_ref;
    }

    void set_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        m_inverse_inertia_tensor_ref = sanitize_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);
        if (auto* body = physics_body(); body != nullptr) {
            body->set_inverse_inertia_tensor_ref(m_inverse_inertia_tensor_ref);
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
