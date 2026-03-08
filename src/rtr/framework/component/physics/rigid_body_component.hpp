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
    pbpt::math::Float              m_mass{1.0f};
    bool                           m_use_gravity{true};
    bool                           m_registered{false};

    bool has_registered_body() const { return m_registered && m_physics_world.has_rigid_body(m_rigid_body_id); }

    static pbpt::math::Float sanitize_mass(pbpt::math::Float mass) {
        if (!std::isfinite(mass) || mass <= 0.0f) {
            throw std::invalid_argument("RigidBody mass must be finite and positive.");
        }
        return mass;
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
                       bool use_gravity = true)
        : Component(owner), m_physics_world(world), m_mass(sanitize_mass(mass)), m_use_gravity(use_gravity) {}

    void on_awake() override {}

    void on_enable() override {
        if (m_registered) {
            return;
        }

        system::physics::RigidBody body;
        body.set_type(system::physics::RigidBodyType::Dynamic);
        body.set_awake(true);
        body.set_use_gravity(m_use_gravity);
        body.state().mass                        = m_mass;
        body.state().translation.position        = owner().node().world_position();
        body.state().translation.linear_velocity = pbpt::math::Vec3(0.0f);
        body.clear_forces();
        body.invalidate_integrator_state();

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
        body->invalidate_integrator_state();
    }

    pbpt::math::Vec3 linear_velocity() const {
        const auto* body = physics_body();
        if (body == nullptr) {
            return pbpt::math::Vec3(0.0f);
        }
        return body->state().translation.linear_velocity;
    }

    pbpt::math::Float mass() const {
        const auto* body = physics_body();
        return body != nullptr ? body->state().mass : m_mass;
    }

    void set_mass(pbpt::math::Float mass) {
        m_mass = sanitize_mass(mass);
        if (auto* body = physics_body(); body != nullptr) {
            body->state().mass = m_mass;
            body->invalidate_integrator_state();
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

    void add_force(const pbpt::math::Vec3& force) {
        if (auto* body = physics_body(); body != nullptr) {
            body->state().forces.accumulated_force += force;
        }
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
};

}  // namespace rtr::framework::component
