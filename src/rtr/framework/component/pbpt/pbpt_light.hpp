#pragma once

#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

struct PbptAreaEmitter {
    PbptSpectrum radiance_spectrum{make_constant_pbpt_spectrum(1.0f)};
};

class PbptLight final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.pbpt_light");
    }

    PbptAreaEmitter m_area_emitter{};

    const MeshRenderer& require_mesh_renderer() const {
        const auto* go = owner();
        if (go == nullptr) {
            logger()->error("PbptLight require_mesh_renderer failed: owner is null.");
            throw std::runtime_error("PbptLight owner is null.");
        }

        const auto* renderer = go->get_component<MeshRenderer>();
        if (renderer == nullptr) {
            logger()->error(
                "PbptLight require_mesh_renderer failed: owner {} missing MeshRenderer.",
                go->id()
            );
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

    resource::MeshHandle mesh_handle() const {
        return require_mesh_renderer().mesh_handle();
    }

    const PbptAreaEmitter& area_emitter() const {
        return m_area_emitter;
    }

    void set_radiance_spectrum(PbptSpectrum points) {
        validate_pbpt_spectrum(points, "PbptLight.radiance_spectrum");
        m_area_emitter.radiance_spectrum = std::move(points);
        logger()->debug("PbptLight radiance_spectrum updated.");
    }
};

} // namespace rtr::framework::component
