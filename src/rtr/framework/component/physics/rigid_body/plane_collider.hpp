#pragma once

#include <cmath>
#include <stdexcept>

#include <pbpt/math/math.h>

#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"

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
    explicit PlaneCollider(core::GameObject& owner, system::physics::RigidBodyWorld& world,
                           const pbpt::math::Vec3& normal_local = pbpt::math::Vec3{0.0f, 1.0f, 0.0f},
                           const pbpt::math::Vec3& local_position = pbpt::math::Vec3{0.0f},
                           const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity())
        : Collider(owner, world),
          m_normal_local(sanitize_normal(normal_local)),
          m_local_position(sanitize_position(local_position)),
          m_local_rotation(sanitize_rotation(local_rotation)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        auto& rigid_body = owner_rigid_body_or_throw();
        if (!has_registered_collider()) {
            system::physics::Collider collider;
            collider.shape = system::physics::PlaneShape{.normal_local = m_normal_local};
            m_collider_id = m_physics_world.create_collider(rigid_body.rigid_body_id(), std::move(collider));
            m_registered = true;
        }
        sync_to_physics();
    }

    void on_disable() override {
        if (!m_registered) {
            return;
        }
        (void)m_physics_world.remove_collider(m_collider_id);
        m_collider_id = system::physics::kInvalidColliderId;
        m_registered = false;
    }

    void on_destroy() override {}

    void sync_to_physics() override {
        if (!has_registered_collider()) {
            auto* rigid_body = owner().get_component<RigidBody>();
            if (rigid_body == nullptr || !rigid_body->has_rigid_body()) {
                return;
            }
            system::physics::Collider collider;
            collider.shape = system::physics::PlaneShape{.normal_local = m_normal_local};
            m_collider_id = m_physics_world.create_collider(rigid_body->rigid_body_id(), std::move(collider));
            m_registered = true;
        }

        auto* collider = physics_collider();
        if (collider == nullptr) {
            return;
        }

        collider->shape = system::physics::PlaneShape{.normal_local = m_normal_local};
        collider->local_transform.position = m_local_position;
        collider->local_transform.rotation = m_local_rotation;
        collider->local_transform.scale = pbpt::math::Vec3{1.0f};
    }

    pbpt::math::Vec3 normal_local() const { return m_normal_local; }
    void set_normal_local(const pbpt::math::Vec3& normal_local) {
        m_normal_local = sanitize_normal(normal_local);
        sync_to_physics();
    }

    pbpt::math::Vec3 local_position() const { return m_local_position; }
    void set_local_position(const pbpt::math::Vec3& local_position) {
        m_local_position = sanitize_position(local_position);
        sync_to_physics();
    }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
        sync_to_physics();
    }
};

}  // namespace rtr::framework::component
