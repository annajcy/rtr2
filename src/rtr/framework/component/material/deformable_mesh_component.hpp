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
        return system::render::MeshView{
            .vertex_buffer = dm.vertex_buffer(),
            .index_buffer  = dm.index_buffer(),
            .index_count   = dm.index_count()
        };
    }

    void apply_deformed_surface(std::span<const pbpt::math::Vec3> positions,
                                std::span<const pbpt::math::Vec3> normals) {
        if (!m_resources.alive<resource::DeformableMeshResourceKind>(m_mesh)) {
            throw std::runtime_error("DeformableMeshComponent mesh handle is not alive.");
        }

        const auto& current_mesh = m_resources.cpu<resource::DeformableMeshResourceKind>(m_mesh);
        const auto vertex_count = current_mesh.vertices.size();
        if (positions.size() != vertex_count || normals.size() != vertex_count) {
            throw std::invalid_argument("DeformableMeshComponent::apply_deformed_surface size mismatch.");
        }

        m_resources.update_cpu<resource::DeformableMeshResourceKind>(m_mesh, [&](auto& mesh) {
            for (std::size_t i = 0; i < vertex_count; ++i) {
                mesh.vertices[i].position = positions[i];
                mesh.vertices[i].normal   = normals[i];
            }
        });
    }
};

} // namespace rtr::framework::component
