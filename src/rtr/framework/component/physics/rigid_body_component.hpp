#pragma once

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::component {

class RigidBody final : public Component {
private:
    system::physics::PhysicsWorld& m_physics_world;
    system::physics::RigidBodyID   m_rigid_body_id{};
    pbpt::math::Vec3               m_velocity{};
    bool                           m_registered{false};

    bool has_registered_body() const { return m_registered && m_physics_world.has_rigid_body(m_rigid_body_id); }

public:
    RigidBody(core::GameObject& owner, system::physics::PhysicsWorld& world, pbpt::math::Vec3 velocity)
        : Component(owner), m_physics_world(world), m_velocity(velocity) {}

    void on_awake() override {}

    void on_enable() override {
        if (m_registered) {
            return;
        }
        m_rigid_body_id                         = m_physics_world.create_rigid_body();
        auto& state                             = m_physics_world.get_rigid_body(m_rigid_body_id).state();
        state.translation_state.position        = owner().node().world_position();
        state.translation_state.linear_velocity = m_velocity;
        m_registered                            = true;
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
        if (!has_registered_body()) {
            return pbpt::math::Vec3(0.0f);
        }
        return m_physics_world.get_rigid_body(m_rigid_body_id).state().translation_state.position;
    }

    void set_position(const pbpt::math::Vec3& position) {
        if (!has_registered_body()) {
            return;
        }
        m_physics_world.get_rigid_body(m_rigid_body_id).state().translation_state.position = position;
    }

    pbpt::math::Vec3 linear_velocity() const {
        if (!has_registered_body()) {
            return pbpt::math::Vec3(0.0f);
        }
        return m_physics_world.get_rigid_body(m_rigid_body_id).state().translation_state.linear_velocity;
    }

    void set_linear_velocity(const pbpt::math::Vec3& linear_velocity) {
        if (!has_registered_body()) {
            return;
        }
        m_physics_world.get_rigid_body(m_rigid_body_id).state().translation_state.linear_velocity = linear_velocity;
    }
};

}  // namespace rtr::framework::component
