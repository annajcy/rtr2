#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/game_object.hpp"

namespace rtr::framework::component {

struct PbptDiffuseBsdf {
    PbptSpectrum reflectance_spectrum{make_constant_pbpt_spectrum(0.7f)};
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

    const PbptSpectrum& reflectance_spectrum() const {
        return m_diffuse_bsdf.reflectance_spectrum;
    }

    void set_reflectance_spectrum(PbptSpectrum points) {
        validate_pbpt_spectrum(points, "PbptMesh.reflectance_spectrum");
        m_diffuse_bsdf.reflectance_spectrum = std::move(points);
    }
};

} // namespace rtr::framework::component
