#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/collider_component.hpp"
#include "rtr/framework/component/physics/rigid_body_component.hpp"

namespace rtr::framework::component {

class SphereCollider final : public ColliderComponent {
private:
    pbpt::math::Float m_radius{0.5f};
    pbpt::math::Vec3  m_local_center{0.0f};

    static pbpt::math::Float sanitize_radius(pbpt::math::Float radius) {
        if (!std::isfinite(radius) || radius <= 0.0f) {
            throw std::invalid_argument("SphereCollider radius must be finite and positive.");
        }
        return radius;
    }

    static pbpt::math::Vec3 sanitize_center(const pbpt::math::Vec3& center) {
        if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z())) {
            throw std::invalid_argument("SphereCollider local_center must be finite.");
        }
        return center;
    }

    static pbpt::math::Vec3 hadamard(const pbpt::math::Vec3& lhs, const pbpt::math::Vec3& rhs) {
        return pbpt::math::Vec3{lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z()};
    }

public:
    explicit SphereCollider(core::GameObject& owner, system::physics::PhysicsWorld& world,
                            pbpt::math::Float radius = 0.5f, const pbpt::math::Vec3& local_center = pbpt::math::Vec3{0.0f})
        : ColliderComponent(owner, world),
          m_radius(sanitize_radius(radius)),
          m_local_center(sanitize_center(local_center)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        if (m_registered) {
            sync_to_physics();
            return;
        }

        system::physics::Collider collider;
        collider.shape = system::physics::SphereShape{.radius = m_radius};
        m_collider_id  = m_physics_world.create_collider(std::move(collider));
        m_registered   = true;
        sync_to_physics();
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_world.remove_collider(m_collider_id);
        m_collider_id = system::physics::ColliderID{};
        m_registered  = false;
    }

    void on_destroy() override {}

    void sync_to_physics() override {
        auto* collider = physics_collider();
        if (collider == nullptr) {
            return;
        }

        const auto node_scale = owner().node().world_scale();
        const auto* rigid_body = owner().get_component<RigidBody>();
        const bool  bound_to_body = rigid_body != nullptr && rigid_body->has_rigid_body();

        collider->shape         = system::physics::SphereShape{.radius = m_radius};
        collider->local_center  = m_local_center;
        collider->local_rotation = pbpt::math::Quat::identity();
        collider->world_scale   = node_scale;
        collider->rigid_body_id = bound_to_body ? std::optional{rigid_body->rigid_body_id()} : std::nullopt;

        const pbpt::math::Vec3 anchor_position =
            bound_to_body ? rigid_body->position() : owner().node().world_position();
        const pbpt::math::Quat anchor_rotation =
            bound_to_body ? rigid_body->orientation() : owner().node().world_rotation();
        collider->world_position = anchor_position + anchor_rotation * hadamard(m_local_center, node_scale);
        collider->world_rotation = anchor_rotation;
    }

    pbpt::math::Float radius() const { return m_radius; }
    void set_radius(pbpt::math::Float radius) {
        m_radius = sanitize_radius(radius);
        sync_to_physics();
    }

    pbpt::math::Vec3 local_center() const { return m_local_center; }
    void set_local_center(const pbpt::math::Vec3& local_center) {
        m_local_center = sanitize_center(local_center);
        sync_to_physics();
    }
};

}  // namespace rtr::framework::component
