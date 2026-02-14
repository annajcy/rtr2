#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/game_object.hpp"

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
        throw std::invalid_argument(std::string(field_name) + " must be finite.");
    }
    if (rgb.r < 0.0f || rgb.r > 1.0f ||
        rgb.g < 0.0f || rgb.g > 1.0f ||
        rgb.b < 0.0f || rgb.b > 1.0f) {
        throw std::invalid_argument(std::string(field_name) + " channels must be in [0, 1].");
    }
}

using PbptReflectance = std::variant<PbptSpectrum, PbptRgb>;

struct PbptDiffuseBsdf {
    PbptReflectance reflectance{make_constant_pbpt_spectrum(0.7f)};
};

class PbptMesh final : public Component {
private:
    PbptDiffuseBsdf m_diffuse_bsdf{};

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

    resource::MeshHandle mesh_handle() const {
        return require_mesh_renderer().mesh_handle();
    }

    const PbptDiffuseBsdf& diffuse_bsdf() const {
        return m_diffuse_bsdf;
    }

    const PbptReflectance& reflectance() const {
        return m_diffuse_bsdf.reflectance;
    }

    bool is_reflectance_spectrum() const {
        return std::holds_alternative<PbptSpectrum>(m_diffuse_bsdf.reflectance);
    }

    bool is_reflectance_rgb() const {
        return std::holds_alternative<PbptRgb>(m_diffuse_bsdf.reflectance);
    }

    const PbptSpectrum& reflectance_spectrum() const {
        const auto* spectrum = std::get_if<PbptSpectrum>(&m_diffuse_bsdf.reflectance);
        if (spectrum == nullptr) {
            throw std::logic_error("PbptMesh reflectance is not a spectrum.");
        }
        return *spectrum;
    }

    const PbptRgb& reflectance_rgb() const {
        const auto* rgb = std::get_if<PbptRgb>(&m_diffuse_bsdf.reflectance);
        if (rgb == nullptr) {
            throw std::logic_error("PbptMesh reflectance is not rgb.");
        }
        return *rgb;
    }

    void set_reflectance_spectrum(PbptSpectrum points) {
        validate_pbpt_spectrum(points, "PbptMesh.reflectance_spectrum");
        m_diffuse_bsdf.reflectance = std::move(points);
    }

    void set_reflectance_rgb(PbptRgb rgb) {
        validate_pbpt_rgb(rgb, "PbptMesh.reflectance_rgb");
        m_diffuse_bsdf.reflectance = rgb;
    }
};

} // namespace rtr::framework::component
