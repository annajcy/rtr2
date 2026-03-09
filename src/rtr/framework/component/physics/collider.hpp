#pragma once

#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/physics_world.hpp"

namespace rtr::framework::component {

class Collider : public Component {
protected:
    system::physics::PhysicsWorld& m_physics_world;
    system::physics::ColliderID    m_collider_id{system::physics::kInvalidColliderId};
    bool                           m_registered{false};

    bool has_registered_collider() const { return m_registered && m_physics_world.has_collider(m_collider_id); }

    system::physics::Collider* physics_collider() {
        if (!has_registered_collider()) {
            return nullptr;
        }
        return &m_physics_world.get_collider(m_collider_id);
    }

    const system::physics::Collider* physics_collider() const {
        if (!has_registered_collider()) {
            return nullptr;
        }
        return &m_physics_world.get_collider(m_collider_id);
    }

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
    explicit Collider(core::GameObject& owner, system::physics::PhysicsWorld& world)
        : Component(owner), m_physics_world(world) {}

    bool has_collider() const { return has_registered_collider(); }

    virtual void sync_to_physics() = 0;
};

}  // namespace rtr::framework::component
