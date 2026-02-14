#pragma once

#include <glm/vec4.hpp>

#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/resource/resource_types.hpp"

namespace rtr::framework::component {

class MeshRenderer final : public Component {
private:
    resource::MeshHandle m_mesh{};
    glm::vec4 m_base_color{1.0f, 1.0f, 1.0f, 1.0f};

public:
    explicit MeshRenderer(
        resource::MeshHandle mesh,
        glm::vec4 base_color = glm::vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : m_mesh(mesh),
          m_base_color(base_color) {
        if (!m_mesh.is_valid()) {
            throw std::invalid_argument("MeshRenderer mesh handle must be valid.");
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

    const glm::vec4& base_color() const {
        return m_base_color;
    }

    void set_base_color(const glm::vec4& base_color) {
        m_base_color = base_color;
    }
};

} // namespace rtr::framework::component
