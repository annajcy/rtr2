#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/collider.hpp"

namespace rtr::framework::component {

class SphereCollider final : public Collider {
private:
    pbpt::math::Float m_radius{0.5f};
    pbpt::math::Vec3 m_local_position{0.0f};
    pbpt::math::Quat m_local_rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 m_local_scale{1.0f};

    static pbpt::math::Float sanitize_radius(pbpt::math::Float radius) {
        if (!std::isfinite(radius) || radius <= 0.0f) {
            throw std::invalid_argument("SphereCollider radius must be finite and positive.");
        }
        return radius;
    }

    static pbpt::math::Vec3 sanitize_center(const pbpt::math::Vec3& center) {
        if (!std::isfinite(center.x()) || !std::isfinite(center.y()) || !std::isfinite(center.z())) {
            throw std::invalid_argument("SphereCollider local_position must be finite.");
        }
        return center;
    }

    static pbpt::math::Quat sanitize_rotation(const pbpt::math::Quat& rotation) {
        if (!std::isfinite(rotation.w()) || !std::isfinite(rotation.x()) || !std::isfinite(rotation.y()) ||
            !std::isfinite(rotation.z())) {
            throw std::invalid_argument("SphereCollider local_rotation must be finite.");
        }
        return pbpt::math::normalize(rotation);
    }

    static pbpt::math::Vec3 sanitize_scale(const pbpt::math::Vec3& scale) {
        if (!std::isfinite(scale.x()) || !std::isfinite(scale.y()) || !std::isfinite(scale.z()) ||
            scale.x() <= 0.0f || scale.y() <= 0.0f || scale.z() <= 0.0f) {
            throw std::invalid_argument("SphereCollider local_scale must be finite and positive.");
        }
        return scale;
    }

public:
    explicit SphereCollider(core::GameObject& owner, pbpt::math::Float radius = 0.5f,
                            const pbpt::math::Vec3& local_position = pbpt::math::Vec3{0.0f},
                            const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity(),
                            const pbpt::math::Vec3& local_scale = pbpt::math::Vec3{1.0f})
        : Collider(owner),
          m_radius(sanitize_radius(radius)),
          m_local_position(sanitize_center(local_position)),
          m_local_rotation(sanitize_rotation(local_rotation)),
          m_local_scale(sanitize_scale(local_scale)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        (void)owner_rigid_body_or_throw();
        m_should_exist_in_runtime = true;
        m_lifecycle_dirty = true;
    }

    void on_disable() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }
    void on_destroy() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }

    pbpt::math::Float radius() const { return m_radius; }
    void set_radius(pbpt::math::Float radius) {
        m_radius = sanitize_radius(radius);
        m_shape_dirty = true;
    }

    pbpt::math::Vec3 local_position() const { return m_local_position; }
    void set_local_position(const pbpt::math::Vec3& local_position) {
        m_local_position = sanitize_center(local_position);
        m_transform_dirty = true;
    }

    pbpt::math::Vec3 local_center() const { return local_position(); }
    void set_local_center(const pbpt::math::Vec3& local_center) { set_local_position(local_center); }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
        m_transform_dirty = true;
    }

    pbpt::math::Vec3 local_scale() const { return m_local_scale; }
    void set_local_scale(const pbpt::math::Vec3& local_scale) {
        m_local_scale = sanitize_scale(local_scale);
        m_transform_dirty = true;
    }
};

}  // namespace rtr::framework::component
