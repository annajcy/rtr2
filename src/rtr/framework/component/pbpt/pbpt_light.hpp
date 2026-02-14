#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/game_object.hpp"

namespace rtr::framework::component {

struct PbptAreaEmitter {
    PbptSpectrum radiance_spectrum{make_constant_pbpt_spectrum(1.0f)};
};

class PbptLight final : public Component {
private:
    PbptAreaEmitter m_area_emitter{};

    const MeshRenderer& require_mesh_renderer() const {
        const auto* go = owner();
        if (go == nullptr) {
            throw std::runtime_error("PbptLight owner is null.");
        }

        const auto* renderer = go->get_component<MeshRenderer>();
        if (renderer == nullptr) {
            throw std::runtime_error("PbptLight requires MeshRenderer on the same GameObject.");
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

    const PbptAreaEmitter& area_emitter() const {
        return m_area_emitter;
    }

    void set_radiance_spectrum(PbptSpectrum points) {
        validate_pbpt_spectrum(points, "PbptLight.radiance_spectrum");
        m_area_emitter.radiance_spectrum = std::move(points);
    }
};

} // namespace rtr::framework::component
