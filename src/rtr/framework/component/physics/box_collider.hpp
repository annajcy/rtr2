#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"

namespace rtr::framework::component {

class BoxCollider final : public Collider {
private:
    pbpt::math::Vec3 m_half_extents{0.5f, 0.5f, 0.5f};
    pbpt::math::Vec3 m_local_center{0.0f};
    pbpt::math::Quat m_local_rotation{pbpt::math::Quat::identity()};

    static pbpt::math::Vec3 sanitize_half_extents(const pbpt::math::Vec3& half_extents) {
        if (!std::isfinite(half_extents.x()) || !std::isfinite(half_extents.y()) || !std::isfinite(half_extents.z()) ||
            half_extents.x() <= 0.0f || half_extents.y() <= 0.0f || half_extents.z() <= 0.0f) {
            throw std::invalid_argument("BoxCollider half_extents must be finite and positive.");
        }
        return half_extents;
    }

    static pbpt::math::Vec3 sanitize_center(const pbpt::math::Vec3& center) {
        if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z())) {
            throw std::invalid_argument("BoxCollider local_center must be finite.");
        }
        return center;
    }

    static pbpt::math::Quat sanitize_rotation(const pbpt::math::Quat& rotation) {
        if (!std::isfinite(rotation.w()) || !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
            !std::isfinite(rotation.z())) {
            throw std::invalid_argument("BoxCollider local_rotation must be finite.");
        }
        return pbpt::math::normalize(rotation);
    }

public:
    explicit BoxCollider(core::GameObject& owner, system::physics::PhysicsWorld& world,
                         const pbpt::math::Vec3& half_extents = pbpt::math::Vec3{0.5f, 0.5f, 0.5f},
                         const pbpt::math::Vec3& local_center = pbpt::math::Vec3{0.0f},
                         const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity())
        : Collider(owner, world),
          m_half_extents(sanitize_half_extents(half_extents)),
          m_local_center(sanitize_center(local_center)),
          m_local_rotation(sanitize_rotation(local_rotation)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        auto& rigid_body = owner_rigid_body_or_throw();
        if (!has_registered_collider()) {
            system::physics::Collider collider;
            collider.shape      = system::physics::BoxShape{.half_extents = m_half_extents};
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
            collider.shape = system::physics::BoxShape{.half_extents = m_half_extents};
            m_collider_id  = m_physics_world.create_collider(rigid_body->rigid_body_id(), std::move(collider));
            m_registered   = true;
        }

        auto* collider = physics_collider();
        if (collider == nullptr) {
            return;
        }

        collider->shape                    = system::physics::BoxShape{.half_extents = m_half_extents};
        collider->local_transform.position = m_local_center;
        collider->local_transform.rotation = m_local_rotation;
        collider->local_transform.scale    = pbpt::math::Vec3(1.0f);
    }

    pbpt::math::Vec3 half_extents() const { return m_half_extents; }
    void set_half_extents(const pbpt::math::Vec3& half_extents) {
        m_half_extents = sanitize_half_extents(half_extents);
        sync_to_physics();
    }

    pbpt::math::Vec3 local_center() const { return m_local_center; }
    void set_local_center(const pbpt::math::Vec3& local_center) {
        m_local_center = sanitize_center(local_center);
        sync_to_physics();
    }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
        sync_to_physics();
    }
};

}  // namespace rtr::framework::component
