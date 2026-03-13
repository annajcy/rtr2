#pragma once

#include <pbpt/math/math.h>
#include <span>
#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class DeformableMeshRenderer final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.deformable_mesh_renderer");
    }

    resource::ResourceManager& m_resources;
    resource::DeformableMeshHandle m_mesh{};
    pbpt::math::Vec4 m_base_color{1.0f, 1.0f, 1.0f, 1.0f};

public:
    explicit DeformableMeshRenderer(
        core::GameObject& owner,
        resource::ResourceManager& resources,
        resource::DeformableMeshHandle mesh,
        pbpt::math::Vec4 base_color = pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : Component(owner),
          m_resources(resources),
          m_mesh(mesh),
          m_base_color(base_color) {
        if (!m_mesh.is_valid()) {
            logger()->error("DeformableMeshRenderer ctor failed: mesh handle is invalid.");
            throw std::invalid_argument("DeformableMeshRenderer mesh handle must be valid.");
        }
    }

    resource::DeformableMeshHandle mesh_handle() const {
        return m_mesh;
    }

    rhi::DynamicMesh& require_dynamic_mesh(rhi::Device& device) {
        return m_resources.require_gpu<resource::DeformableMeshResourceKind>(m_mesh, device);
    }

    system::render::MeshView mesh_view(rhi::Device& device) {
        auto& dm = require_dynamic_mesh(device);
        return system::render::MeshView{
            .vertex_buffer = dm.vertex_buffer(),
            .index_buffer  = dm.index_buffer(),
            .index_count   = dm.index_count()
        };
    }

    void update_positions(rhi::Device& device, std::span<const rhi::DynamicMesh::Vertex> vertices) {
        require_dynamic_mesh(device).update_vertices(vertices);
    }

    void update_normals(rhi::Device& device, std::span<const pbpt::math::Vec3> normals) {
        require_dynamic_mesh(device).update_normals(normals);
    }

    const pbpt::math::Vec4& base_color() const {
        return m_base_color;
    }

    void set_base_color(const pbpt::math::Vec4& base_color) {
        m_base_color = base_color;
    }
};

} // namespace rtr::framework::component
