#pragma once

#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/rigid_body/rigid_body_system.hpp"

namespace rtr::framework::component {

class Collider : public Component {
protected:
    bool m_shape_dirty{false};
    bool m_transform_dirty{false};
    bool m_lifecycle_dirty{false};
    bool m_should_exist_in_runtime{false};

    void throw_if_owner_already_has_collider() const {
        if (owner().get_component<Collider>() != nullptr) {
            throw std::runtime_error("GameObject already has a collider.");
        }
    }

    framework::component::RigidBody& owner_rigid_body_or_throw() {
        auto* rigid_body = owner().get_component<framework::component::RigidBody>();
        if (rigid_body == nullptr) {
            throw std::runtime_error("Collider requires a RigidBody component.");
        }
        return *rigid_body;
    }

public:
    explicit Collider(core::GameObject& owner) : Component(owner) {}

    bool shape_dirty() const { return m_shape_dirty; }
    bool transform_dirty() const { return m_transform_dirty; }
    bool lifecycle_dirty() const { return m_lifecycle_dirty; }
    bool should_exist_in_runtime() const { return m_should_exist_in_runtime; }

    void clear_shape_dirty() { m_shape_dirty = false; }
    void clear_transform_dirty() { m_transform_dirty = false; }
    void clear_lifecycle_dirty() { m_lifecycle_dirty = false; }
};

}  // namespace rtr::framework::component
