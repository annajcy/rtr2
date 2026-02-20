#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "pbpt/integrator/plugin/integrator/integrator_type.hpp"
#include "pbpt/scene/scene.hpp"

#include "rtr/framework/core/types.hpp"

namespace rtr::framework::integration {

struct PbptMappedShapeInfo {
    std::string source_shape_id{};
    std::string source_mesh_name{};
    std::string source_material_ref_name{};
};

struct PbptCompatibleInfo {
    // Runtime mapping for RTR-owned objects that came from PBPT shapes.
    std::unordered_map<core::GameObjectId, PbptMappedShapeInfo> mapped_shape_info_by_game_object{};

    // PBPT resources preserved for scene-level passthrough of unmapped content.
    pbpt::scene::RenderResources<float> passthrough_resources{};
    std::unordered_set<std::string> passthrough_shape_ids{};

    // Optional passthrough integrator metadata for preserving import settings.
    std::optional<pbpt::integrator::AnyIntegrator<float>> passthrough_integrator{};
    int passthrough_spp{4};
};

} // namespace rtr::framework::integration
