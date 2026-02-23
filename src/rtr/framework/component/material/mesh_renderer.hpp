#pragma once

#include <pbpt/math/math.h>


#include <stdexcept>

#include "rtr/framework/component/component.hpp"
#include "rtr/resource/resource_types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class MeshRenderer final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.mesh_renderer");
    }

    resource::MeshHandle m_mesh{};
    pbpt::math::vec4 m_base_color{1.0f, 1.0f, 1.0f, 1.0f};

public:
    explicit MeshRenderer(
        core::GameObject& owner,
        resource::MeshHandle mesh,
        pbpt::math::vec4 base_color = pbpt::math::vec4{1.0f, 1.0f, 1.0f, 1.0f}
    )
        : Component(owner),
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

    const pbpt::math::vec4& base_color() const {
        return m_base_color;
    }

    void set_base_color(const pbpt::math::vec4& base_color) {
        m_base_color = base_color;
    }
};

} // namespace rtr::framework::component
