#pragma once

#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::framework::component {

class Collider : public Component {
protected:
    system::physics::RigidBodyWorld& m_physics_world;
    system::physics::ColliderID    m_collider_id{system::physics::kInvalidColliderId};
    bool                           m_registered{false};

    bool has_registered_collider() const { return m_registered && m_physics_world.has_collider(m_collider_id); }

    void throw_if_owner_already_has_collider() const {
        if (owner().get_component<Collider>() != nullptr) {
            throw std::runtime_error("GameObject already has a collider.");
        }
    }

    framework::component::RigidBody& owner_rigid_body_or_throw() {
        auto* rigid_body = owner().get_component<framework::component::RigidBody>();
        if (rigid_body == nullptr || !rigid_body->has_rigid_body()) {
            throw std::runtime_error("Collider requires an enabled RigidBody.");
        }
        return *rigid_body;
    }

public:
    explicit Collider(core::GameObject& owner, system::physics::RigidBodyWorld& world)
        : Component(owner), m_physics_world(world) {}

    bool has_collider() const { return has_registered_collider(); }
    system::physics::ColliderID collider_id() const { return m_collider_id; }
};

}  // namespace rtr::framework::component
