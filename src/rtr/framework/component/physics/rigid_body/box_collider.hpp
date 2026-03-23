#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"

namespace rtr::framework::component {

class BoxCollider final : public Collider {
private:
    pbpt::math::Vec3 m_half_extents{0.5f, 0.5f, 0.5f};
    pbpt::math::Vec3 m_local_position{0.0f};
    pbpt::math::Quat m_local_rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 m_local_scale{1.0f};

    static pbpt::math::Vec3 sanitize_half_extents(const pbpt::math::Vec3& half_extents) {
        if (!std::isfinite(half_extents.x()) || !std::isfinite(half_extents.y()) || !std::isfinite(half_extents.z()) ||
            half_extents.x() <= 0.0f || half_extents.y() <= 0.0f || half_extents.z() <= 0.0f) {
            throw std::invalid_argument("BoxCollider half_extents must be finite and positive.");
        }
        return half_extents;
    }

    static pbpt::math::Vec3 sanitize_center(const pbpt::math::Vec3& center) {
        if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z())) {
            throw std::invalid_argument("BoxCollider local_position must be finite.");
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

    static pbpt::math::Vec3 sanitize_scale(const pbpt::math::Vec3& scale) {
        if (!std::isfinite(scale.x()) || !std::isfinite(scale.y()) || !std::isfinite(scale.z()) ||
            scale.x() <= 0.0f || scale.y() <= 0.0f || scale.z() <= 0.0f) {
            throw std::invalid_argument("BoxCollider local_scale must be finite and positive.");
        }
        return scale;
    }

public:
    explicit BoxCollider(core::GameObject& owner, system::physics::rb::RigidBodySystem& world,
                         const pbpt::math::Vec3& half_extents = pbpt::math::Vec3{0.5f, 0.5f, 0.5f},
                         const pbpt::math::Vec3& local_position = pbpt::math::Vec3{0.0f},
                         const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity(),
                         const pbpt::math::Vec3& local_scale = pbpt::math::Vec3{1.0f})
        : Collider(owner, world),
          m_half_extents(sanitize_half_extents(half_extents)),
          m_local_position(sanitize_center(local_position)),
          m_local_rotation(sanitize_rotation(local_rotation)),
          m_local_scale(sanitize_scale(local_scale)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        auto& rigid_body = owner_rigid_body_or_throw();
        if (!has_registered_collider()) {
            system::physics::rb::Collider collider;
            collider.shape                    = system::physics::rb::BoxShape{.half_extents = m_half_extents};
            collider.local_transform.position = m_local_position;
            collider.local_transform.rotation = m_local_rotation;
            collider.local_transform.scale    = m_local_scale;
            m_collider_id                     = m_physics_world.create_collider(rigid_body.rigid_body_id(), std::move(collider));
            m_registered                      = true;
        }
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_world.remove_collider(m_collider_id);
        m_collider_id = system::physics::rb::kInvalidColliderId;
        m_registered  = false;
    }

    void on_destroy() override {}

    pbpt::math::Vec3 half_extents() const { return m_half_extents; }
    void set_half_extents(const pbpt::math::Vec3& half_extents) {
        m_half_extents = sanitize_half_extents(half_extents);
    }

    pbpt::math::Vec3 local_position() const { return m_local_position; }
    void set_local_position(const pbpt::math::Vec3& local_position) {
        m_local_position = sanitize_center(local_position);
    }

    pbpt::math::Vec3 local_center() const { return local_position(); }
    void set_local_center(const pbpt::math::Vec3& local_center) {
        set_local_position(local_center);
    }

    pbpt::math::Vec3 local_scale() const { return m_local_scale; }
    void set_local_scale(const pbpt::math::Vec3& local_scale) {
        m_local_scale = sanitize_scale(local_scale);
    }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
    }
};

}  // namespace rtr::framework::component
