#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>

#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/integration/pbpt/pbpt_compatible_info.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_metadata.hpp"

namespace rtr::system::input {
class InputState;
}

namespace rtr::framework::integration {

struct PbptImportOptions {
    bool                             require_supported_cbox_subset{true};
    const system::input::InputState* free_look_input_state{nullptr};
};

struct PbptImportResult {
    std::size_t                                         imported_shape_count{0};
    std::size_t                                         imported_light_shape_count{0};
    std::unordered_map<std::string, core::GameObjectId> imported_game_object_id_by_name{};
    std::optional<PbptIntegratorRecord>                 integrator{};
    std::optional<PbptSensorRecord>                     sensor{};
};

struct PbptImportPackage {
    PbptImportResult   result{};
    PbptCompatibleInfo compatible_info{};
};

}  // namespace rtr::framework::integration
