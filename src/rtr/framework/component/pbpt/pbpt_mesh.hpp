#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

struct PbptRgb {
    float r{0.0f};
    float g{0.0f};
    float b{0.0f};
};

inline void validate_pbpt_rgb(
    const PbptRgb& rgb,
    std::string_view field_name = "rgb"
) {
    if (!std::isfinite(rgb.r) || !std::isfinite(rgb.g) || !std::isfinite(rgb.b)) {
        utils::get_logger("framework.component.pbpt_mesh")->error(
            "{} validation failed: rgb contains non-finite values.",
            field_name
        );
        throw std::invalid_argument(std::string(field_name) + " must be finite.");
    }
    if (rgb.r < 0.0f || rgb.r > 1.0f ||
        rgb.g < 0.0f || rgb.g > 1.0f ||
        rgb.b < 0.0f || rgb.b > 1.0f) {
        utils::get_logger("framework.component.pbpt_mesh")->error(
            "{} validation failed: rgb channels out of [0,1].",
            field_name
        );
        throw std::invalid_argument(std::string(field_name) + " channels must be in [0, 1].");
    }
}

class PbptMesh final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.pbpt_mesh");
    }

    const MeshRenderer& require_mesh_renderer() const { return owner().component_or_throw<MeshRenderer>(); }

public:
    explicit PbptMesh(core::GameObject& owner)
        : Component(owner) {}

    void on_awake() override {
        (void)require_mesh_renderer();
    }

    const MeshRenderer& mesh_renderer() const {
        return require_mesh_renderer();
    }

    resource::MeshHandle mesh_handle() const {
        return require_mesh_renderer().mesh_handle();
    }
};

} // namespace rtr::framework::component
