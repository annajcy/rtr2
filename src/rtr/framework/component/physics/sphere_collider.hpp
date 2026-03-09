#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"

namespace rtr::framework::component {

class SphereCollider final : public Collider {
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

public:
    explicit SphereCollider(core::GameObject& owner, system::physics::PhysicsWorld& world,
                            pbpt::math::Float radius = 0.5f, const pbpt::math::Vec3& local_center = pbpt::math::Vec3{0.0f})
        : Collider(owner, world),
          m_radius(sanitize_radius(radius)),
          m_local_center(sanitize_center(local_center)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        auto& rigid_body = owner_rigid_body_or_throw();
        if (!has_registered_collider()) {
            system::physics::Collider collider;
            collider.shape      = system::physics::SphereShape{.radius = m_radius};
            m_collider_id       = m_physics_world.create_collider(rigid_body.rigid_body_id(), std::move(collider));
            m_registered        = true;
        }
        sync_to_physics();
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_world.remove_collider(m_collider_id);
        m_collider_id = system::physics::kInvalidColliderId;
        m_registered  = false;
    }

    void on_destroy() override {}

    void sync_to_physics() override {
        if (!has_registered_collider()) {
            auto* rigid_body = owner().get_component<RigidBody>();
            if (rigid_body == nullptr || !rigid_body->has_rigid_body()) {
                return;
            }
            system::physics::Collider collider;
            collider.shape = system::physics::SphereShape{.radius = m_radius};
            m_collider_id  = m_physics_world.create_collider(rigid_body->rigid_body_id(), std::move(collider));
            m_registered   = true;
        }

        auto* collider = physics_collider();
        if (collider == nullptr) {
            return;
        }

        collider->shape                    = system::physics::SphereShape{.radius = m_radius};
        collider->local_transform.position = m_local_center;
        collider->local_transform.rotation = pbpt::math::Quat::identity();
        collider->local_transform.scale    = pbpt::math::Vec3(1.0f);
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
