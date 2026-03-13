#pragma once

#include <pbpt/math/math.h>
#include <stdexcept>
#include <vector>

#include "rtr/framework/component/material/mesh_component.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class StaticMeshComponent final : public MeshComponent {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.static_mesh_component");
    }

    resource::ResourceManager& m_resources;
    resource::MeshHandle       m_mesh{};

public:
    explicit StaticMeshComponent(
        core::GameObject& owner,
        resource::ResourceManager& resources,
        resource::MeshHandle mesh,
        pbpt::math::Vec4 base_color = pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : MeshComponent(owner, base_color),
          m_resources(resources),
          m_mesh(mesh) {
        if (!m_mesh.is_valid()) {
            logger()->error("StaticMeshComponent ctor failed: mesh handle is invalid.");
            throw std::invalid_argument("StaticMeshComponent mesh handle must be valid.");
        }
    }

    resource::MeshHandle mesh_handle() const {
        return m_mesh;
    }

    std::vector<pbpt::math::Vec3> local_vertices() const override {
        if (!m_resources.alive<resource::MeshResourceKind>(m_mesh)) {
            throw std::runtime_error("StaticMeshComponent mesh handle is not alive in ResourceManager.");
        }
        const auto& mesh = m_resources.cpu<resource::MeshResourceKind>(m_mesh);
        std::vector<pbpt::math::Vec3> local_vertices;
        local_vertices.reserve(mesh.vertices.size());
        for (const auto& vertex : mesh.vertices) {
            local_vertices.push_back(vertex.position);
        }
        return local_vertices;
    }

    bool has_valid_mesh() const override {
        return m_mesh.is_valid() && m_resources.alive<resource::MeshResourceKind>(m_mesh);
    }

    system::render::MeshView mesh_view(rhi::Device& device) override {
        auto& mesh = m_resources.require_gpu<resource::MeshResourceKind>(m_mesh, device);
        return system::render::MeshView{
            .vertex_buffer = mesh.vertex_buffer(),
            .index_buffer  = mesh.index_buffer(),
            .index_count   = mesh.index_count()
        };
    }

    void set_mesh_handle(resource::MeshHandle mesh) {
        if (!mesh.is_valid()) {
            logger()->error("set_mesh_handle failed: mesh handle is invalid.");
            throw std::invalid_argument("StaticMeshComponent mesh handle must be valid.");
        }
        if (m_mesh.value != mesh.value) {
            logger()->debug("StaticMeshComponent mesh handle updated (old={}, new={}).", m_mesh.value, mesh.value);
        }
        m_mesh = mesh;
    }
};

} // namespace rtr::framework::component
