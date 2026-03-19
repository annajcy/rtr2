#pragma once

#include <stdexcept>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/framework/component/material/mesh_component.hpp"
#include "rtr/framework/component/physics/rigid_body/collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::component {

class MeshCollider final : public Collider {
private:
    pbpt::math::Vec3           m_local_position{0.0f};
    pbpt::math::Quat           m_local_rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3           m_local_scale{1.0f};

    static void validate_mesh_handle(const resource::MeshHandle& mesh_handle) {
        if (!mesh_handle.is_valid()) {
            throw std::invalid_argument("MeshCollider requires a valid MeshHandle.");
        }
    }

    MeshComponent& owner_mesh_component_or_throw() {
        auto* mesh_component = owner().get_component<MeshComponent>();
        if (mesh_component == nullptr) {
            throw std::runtime_error("MeshCollider requires a MeshComponent on the same GameObject.");
        }
        return *mesh_component;
    }

    const MeshComponent& owner_mesh_component_or_throw() const {
        const auto* mesh_component = owner().get_component<MeshComponent>();
        if (mesh_component == nullptr) {
            throw std::runtime_error("MeshCollider requires a MeshComponent on the same GameObject.");
        }
        return *mesh_component;
    }

    static pbpt::math::Vec3 sanitize_position(const pbpt::math::Vec3& local_position) {
        if (!std::isfinite(local_position.x()) || !std::isfinite(local_position.y()) || !std::isfinite(local_position.z())) {
            throw std::invalid_argument("MeshCollider local_position must be finite.");
        }
        return local_position;
    }

    static pbpt::math::Quat sanitize_rotation(const pbpt::math::Quat& local_rotation) {
        if (!std::isfinite(local_rotation.w()) || !std::isfinite(local_rotation.x()) ||
            !std::isfinite(local_rotation.y()) || !std::isfinite(local_rotation.z())) {
            throw std::invalid_argument("MeshCollider local_rotation must be finite.");
        }
        return pbpt::math::normalize(local_rotation);
    }

    static pbpt::math::Vec3 sanitize_scale(const pbpt::math::Vec3& local_scale) {
        if (!std::isfinite(local_scale.x()) || !std::isfinite(local_scale.y()) || !std::isfinite(local_scale.z()) ||
            local_scale.x() <= 0.0f || local_scale.y() <= 0.0f || local_scale.z() <= 0.0f) {
            throw std::invalid_argument("MeshCollider local_scale must be finite and positive.");
        }
        return local_scale;
    }

    std::vector<pbpt::math::Vec3> local_vertices_from_renderer() const {
        auto local_vertices = owner_mesh_component_or_throw().local_vertices();
        if (local_vertices.empty()) {
            throw std::runtime_error("MeshCollider requires a mesh with at least one vertex.");
        }
        return local_vertices;
    }

public:
    explicit MeshCollider(core::GameObject& owner, system::physics::RigidBodyWorld& world,
                          const pbpt::math::Vec3& local_position = pbpt::math::Vec3{0.0f},
                          const pbpt::math::Quat& local_rotation = pbpt::math::Quat::identity(),
                          const pbpt::math::Vec3& local_scale = pbpt::math::Vec3{1.0f})
        : Collider(owner, world),
          m_local_position(sanitize_position(local_position)),
          m_local_rotation(sanitize_rotation(local_rotation)),
          m_local_scale(sanitize_scale(local_scale)) {}

    void on_awake() override { throw_if_owner_already_has_collider(); }

    void on_enable() override {
        auto& rigid_body = owner_rigid_body_or_throw();
        const auto local_vertices = local_vertices_from_renderer();
        if (!has_registered_collider()) {
            system::physics::Collider collider;
            collider.shape                    = system::physics::MeshShape{.local_vertices = local_vertices};
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
        m_collider_id = system::physics::kInvalidColliderId;
        m_registered = false;
    }

    void on_destroy() override {}

    pbpt::math::Vec3 local_position() const { return m_local_position; }
    void set_local_position(const pbpt::math::Vec3& local_position) {
        m_local_position = sanitize_position(local_position);
    }

    pbpt::math::Quat local_rotation() const { return m_local_rotation; }
    void set_local_rotation(const pbpt::math::Quat& local_rotation) {
        m_local_rotation = sanitize_rotation(local_rotation);
    }

    pbpt::math::Vec3 local_scale() const { return m_local_scale; }
    void set_local_scale(const pbpt::math::Vec3& local_scale) {
        m_local_scale = sanitize_scale(local_scale);
    }
};

}  // namespace rtr::framework::component
