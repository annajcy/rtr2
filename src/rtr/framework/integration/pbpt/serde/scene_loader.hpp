#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/serde/context.hpp"
#include "rtr/framework/integration/pbpt/serde/dispatch.hpp"
#include "rtr/framework/integration/pbpt/serde/load/types.hpp"
#include "rtr/framework/integration/pbpt/serde/domain/typelist.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::integration {

inline std::shared_ptr<spdlog::logger> pbpt_import_logger() {
    return utils::get_logger("framework.integration.pbpt.import");
}

inline LoadPackage load_scene(
    const ::pbpt::serde::PbptXmlResult<float>& pbpt_scene_result, core::Scene& scene,
    resource::ResourceManager& resources, const LoadOptions& options = {}) {
    auto log = pbpt_import_logger();
    LoadPackage package{.result = LoadSummary{},
                        .compatible_info =
                            CompatibleInfo{
                                .mapped_shape_info_by_game_object = {},
                                .passthrough_resources            = pbpt_scene_result.scene.resources,
                                .passthrough_shape_ids            = {},
                                .passthrough_integrator           = std::make_optional(pbpt_scene_result.integrator),
                                .passthrough_spp                  = std::max(1, pbpt_scene_result.spp)}};

    ImportGlobalContext ctx{
        .pbpt_scene_result = pbpt_scene_result, .scene = scene, .resources = resources, .options = options};

    auto integrator_res = dispatch_impl(IntegratorLoaderList{}, pbpt_scene_result.integrator, ctx, package);
    if (!integrator_res.matched) {
        throw std::runtime_error("Unsupported PBPT integrator in import.");
    }

    auto camera_res = dispatch_impl(CameraLoaderList{}, pbpt_scene_result.scene.camera, ctx, package);
    if (!camera_res.matched) {
        throw std::runtime_error("Unsupported PBPT camera in import.");
    }

    for (const auto& shape_record : pbpt_scene_result.scene.resources.shape_instances) {
        try {
            auto shape_res = dispatch_impl(ShapeLoaderList{}, shape_record, ctx, package);
            if (!shape_res.matched) {
                package.compatible_info.passthrough_shape_ids.insert(shape_record.shape_id);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("[domain=shape] ") + e.what());
        }
    }

    scene.scene_graph().update_world_transforms();
    log->info("PBPT import with compatible info completed (mapped_shapes={}, mapped_lights={}, passthrough_shapes={}).",
              package.result.imported_shape_count, package.result.imported_light_shape_count,
              package.compatible_info.passthrough_shape_ids.size());
    return package;
}

inline LoadPackage load_scene(const std::string& scene_xml_path, core::Scene& scene,
                              resource::ResourceManager& resources, const LoadOptions& options = {}) {
    if (scene_xml_path.empty()) {
        throw std::invalid_argument("scene_xml_path must not be empty.");
    }
    try {
        auto pbpt_scene_result = ::pbpt::serde::load_scene<float>(scene_xml_path);
        return load_scene(pbpt_scene_result, scene, resources, options);
    } catch (const std::exception& e) {
        pbpt_import_logger()->error("PBPT XML import failed: {}", e.what());
        throw std::runtime_error(std::string("load_scene failed: ") + e.what());
    }
}

}  // namespace rtr::framework::integration
