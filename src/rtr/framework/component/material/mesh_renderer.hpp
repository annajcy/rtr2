#pragma once

#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/resource/resource_types.hpp"

namespace rtr::framework::component {

class MeshRenderer final : public Component {
private:
    resource::MeshHandle m_mesh{};
    resource::TextureHandle m_albedo_texture{};

public:
    explicit MeshRenderer(
        resource::MeshHandle mesh,
        resource::TextureHandle albedo_texture
    )
        : m_mesh(mesh),
          m_albedo_texture(albedo_texture) {
        if (!m_mesh.is_valid()) {
            throw std::invalid_argument("MeshRenderer mesh handle must be valid.");
        }
        if (!m_albedo_texture.is_valid()) {
            throw std::invalid_argument("MeshRenderer albedo texture handle must be valid.");
        }
    }

    resource::MeshHandle mesh_handle() const {
        return m_mesh;
    }

    void set_mesh_handle(resource::MeshHandle mesh) {
        if (!mesh.is_valid()) {
            throw std::invalid_argument("MeshRenderer mesh handle must be valid.");
        }
        m_mesh = mesh;
    }

    resource::TextureHandle albedo_texture_handle() const {
        return m_albedo_texture;
    }

    void set_albedo_texture_handle(resource::TextureHandle albedo_texture) {
        if (!albedo_texture.is_valid()) {
            throw std::invalid_argument("MeshRenderer albedo texture handle must be valid.");
        }
        m_albedo_texture = albedo_texture;
    }
};

} // namespace rtr::framework::component
