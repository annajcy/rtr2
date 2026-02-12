#pragma once

#include <stdexcept>
#include <string>

#include <glm/common.hpp>
#include <glm/vec3.hpp>

#include "framework/component/component.hpp"
#include "framework/component/mesh_renderer.hpp"
#include "framework/core/game_object.hpp"

namespace rtr::framework::component {

struct PbptDiffuseBsdf {
    glm::vec3 reflectance_rgb{0.7f, 0.7f, 0.7f};
};

class PbptMesh final : public Component {
private:
    PbptDiffuseBsdf m_diffuse_bsdf{};

    static glm::vec3 clamp_rgb01(const glm::vec3& value) {
        return glm::clamp(value, glm::vec3{0.0f}, glm::vec3{1.0f});
    }

    const MeshRenderer& require_mesh_renderer() const {
        const auto* go = owner();
        if (go == nullptr) {
            throw std::runtime_error("PbptMesh owner is null.");
        }

        const auto* renderer = go->get_component<MeshRenderer>();
        if (renderer == nullptr) {
            throw std::runtime_error("PbptMesh requires MeshRenderer on the same GameObject.");
        }

        return *renderer;
    }

public:
    void on_awake() override {
        (void)require_mesh_renderer();
    }

    const MeshRenderer& mesh_renderer() const {
        return require_mesh_renderer();
    }

    const std::string& mesh_path() const {
        return require_mesh_renderer().mesh_path();
    }

    const PbptDiffuseBsdf& diffuse_bsdf() const {
        return m_diffuse_bsdf;
    }

    void set_reflectance_rgb(const glm::vec3& value) {
        m_diffuse_bsdf.reflectance_rgb = clamp_rgb01(value);
    }

    void set_reflectance_rgb(float r, float g, float b) {
        set_reflectance_rgb(glm::vec3{r, g, b});
    }
};

} // namespace rtr::framework::component
