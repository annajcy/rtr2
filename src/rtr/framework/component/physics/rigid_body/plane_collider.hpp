#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/collider.hpp"

namespace rtr::framework::component {

class PlaneCollider final : public Collider {
private:
    pbpt::math::Vec3 m_normal_local{0.0f, 1.0f, 0.0f};
    pbpt::math::Vec3 m_local_position{0.0f};
    pbpt::math::Quat m_local_rotation{pbpt::math::Quat::identity()};

    static pbpt::math::Vec3 sanitize_normal(const pbpt::math::Vec3& normal_local) {
        if (!std::isfinite(normal_local.x()) || !std::isfinite(normal_local.y()) || !std::isfinite(normal_local.z()) ||
            pbpt::math::dot(normal_local, normal_local) <= 1e-12f) {
            throw std::invalid_argument("PlaneCollider normal_local must be finite and non-zero.");
        }
        return pbpt::math::normalize(normal_local);
    }

    static pbpt::math::Vec3 sanitize_position(const pbpt::math::Vec3& local_position) {
        if (!std::isfinite(local_position.x()) || !std::isfinite(local_position.y()) || !std::isfinite(local_position.z())) {
            throw std::invalid_argument("PlaneCollider local_position must be finite.");
        }
        return local_position;
    }

    static pbpt::math::Quat sanitize_rotation(const pbpt::math::Quat& local_rotation) {
        if (!std::isfinite(local_rotation.w()) || !std::isfinite(local_rotation.x()) ||
            !std::isfinite(local_rotation.y()) || !std::isfinite(local_rotation.z())) {
            throw std::invalid_argument("PlaneCollider local_rotation must be finite.");
        }
        return pbpt::math::normalize(local_rotation);
    }

public:
    explicit PlaneCollider(core::GameObject& owner,
                           const pbpt::math::Vec3& normal_local = pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
                           const pbpt::math::Vec3& local_position = pbpt::math::Vec3{0.0f},
                           const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity())
        : Collider(owner),
          m_normal_local(sanitize_normal(normal_local)),
          m_local_position(sanitize_position(local_position)),
          m_local_rotation(sanitize_rotation(local_rotation)) {}

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

    pbpt::math::Vec3 normal_local() const { return m_normal_local; }
    void set_normal_local(const pbpt::math::Vec3& normal_local) {
        m_normal_local = sanitize_normal(normal_local);
        m_shape_dirty = true;
    }

    pbpt::math::Vec3 local_position() const { return m_local_position; }
    void set_local_position(const pbpt::math::Vec3& local_position) {
        m_local_position = sanitize_position(local_position);
        m_transform_dirty = true;
    }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
        m_transform_dirty = true;
    }
};

}  // namespace rtr::framework::component
