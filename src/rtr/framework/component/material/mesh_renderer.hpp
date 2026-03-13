#pragma once

#include <pbpt/math/math.h>
#include <stdexcept>
#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class MeshRenderer final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.mesh_renderer");
    }

    resource::ResourceManager& m_resources;
    resource::MeshHandle       m_mesh{};
    pbpt::math::Vec4           m_base_color{1.0f, 1.0f, 1.0f, 1.0f};

public:
    explicit MeshRenderer(
        core::GameObject& owner,
        resource::ResourceManager& resources,
        resource::MeshHandle mesh,
        pbpt::math::Vec4 base_color = pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : Component(owner),
          m_resources(resources),
          m_mesh(mesh),
          m_base_color(base_color) {
        if (!m_mesh.is_valid()) {
            logger()->error("MeshRenderer ctor failed: mesh handle is invalid.");
            throw std::invalid_argument("MeshRenderer mesh handle must be valid.");
        }
    }

    resource::MeshHandle mesh_handle() const {
        return m_mesh;
    }

    std::vector<pbpt::math::Vec3> local_vertices() const {
        if (!m_resources.alive<resource::MeshResourceKind>(m_mesh)) {
            throw std::runtime_error("MeshRenderer mesh handle is not alive in ResourceManager.");
        }
        const auto& mesh = m_resources.cpu<resource::MeshResourceKind>(m_mesh);
        std::vector<pbpt::math::Vec3> local_vertices;
        local_vertices.reserve(mesh.vertices.size());
        for (const auto& vertex : mesh.vertices) {
            local_vertices.push_back(vertex.position);
        }
        return local_vertices;
    }

    system::render::MeshView mesh_view(rhi::Device& device) {
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
            throw std::invalid_argument("MeshRenderer mesh handle must be valid.");
        }
        if (m_mesh.value != mesh.value) {
            logger()->debug("MeshRenderer mesh handle updated (old={}, new={}).", m_mesh.value, mesh.value);
        }
        m_mesh = mesh;
    }

    const pbpt::math::Vec4& base_color() const {
        return m_base_color;
    }

    void set_base_color(const pbpt::math::Vec4& base_color) {
        m_base_color = base_color;
    }
};

} // namespace rtr::framework::component
