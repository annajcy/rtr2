#pragma once

#include <pbpt/math/math.h>
#include <span>
#include <stdexcept>

#include "rtr/framework/component/material/mesh_component.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class DeformableMeshComponent final : public MeshComponent {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.deformable_mesh_component");
    }

    resource::ResourceManager& m_resources;
    resource::DeformableMeshHandle m_mesh{};

    std::vector<rhi::DynamicMesh::Vertex> m_pending_vertices;
    bool m_vertices_dirty{false};

    std::vector<pbpt::math::Vec3> m_pending_normals;
    bool m_normals_dirty{false};

public:
    explicit DeformableMeshComponent(
        core::GameObject& owner,
        resource::ResourceManager& resources,
        resource::DeformableMeshHandle mesh,
        pbpt::math::Vec4 base_color = pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : MeshComponent(owner, base_color),
          m_resources(resources),
          m_mesh(mesh) {
        if (!m_mesh.is_valid()) {
            logger()->error("DeformableMeshComponent ctor failed: mesh handle is invalid.");
            throw std::invalid_argument("DeformableMeshComponent mesh handle must be valid.");
        }
    }

    resource::DeformableMeshHandle mesh_handle() const {
        return m_mesh;
    }

    std::vector<pbpt::math::Vec3> local_vertices() const override {
        if (!m_resources.alive<resource::DeformableMeshResourceKind>(m_mesh)) {
            throw std::runtime_error("DeformableMeshComponent mesh handle is not alive.");
        }
        const auto& mesh = m_resources.cpu<resource::DeformableMeshResourceKind>(m_mesh);
        std::vector<pbpt::math::Vec3> local_vertices;
        local_vertices.reserve(mesh.vertices.size());
        for (const auto& vertex : mesh.vertices) {
            local_vertices.push_back(vertex.position);
        }
        return local_vertices;
    }

    std::vector<uint32_t> indices() const {
        if (!m_resources.alive<resource::DeformableMeshResourceKind>(m_mesh)) {
            throw std::runtime_error("DeformableMeshComponent mesh handle is not alive.");
        }
        const auto& mesh = m_resources.cpu<resource::DeformableMeshResourceKind>(m_mesh);
        return mesh.indices;
    }

    bool has_valid_mesh() const override {
        return m_mesh.is_valid() && m_resources.alive<resource::DeformableMeshResourceKind>(m_mesh);
    }

    system::render::MeshView mesh_view(rhi::Device& device) override {
        auto& dm = m_resources.require_gpu<resource::DeformableMeshResourceKind>(m_mesh, device);
        
        if (m_vertices_dirty) {
            dm.update_vertices(m_pending_vertices);
            m_vertices_dirty = false;
        }
        if (m_normals_dirty) {
            dm.update_normals(m_pending_normals);
            m_normals_dirty = false;
        }

        return system::render::MeshView{
            .vertex_buffer = dm.vertex_buffer(),
            .index_buffer  = dm.index_buffer(),
            .index_count   = dm.index_count()
        };
    }

    void update_positions(std::span<const rhi::DynamicMesh::Vertex> vertices) {
        m_pending_vertices.assign(vertices.begin(), vertices.end());
        m_vertices_dirty = true;
    }

    void update_normals(std::span<const pbpt::math::Vec3> normals) {
        m_pending_normals.assign(normals.begin(), normals.end());
        m_normals_dirty = true;
    }
};

} // namespace rtr::framework::component
