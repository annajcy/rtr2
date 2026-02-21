#pragma once

#include "pbpt/serde/scene_loader.hpp"

namespace rtr::framework::core {
class Scene;
}

#include "rtr/resource/resource_manager.hpp"
#include <unordered_map>
#include <string>

namespace rtr::framework::integration {

struct PbptImportOptions;
struct PbptCompatibleInfo;

struct ImportGlobalContext {
    const pbpt::serde::PbptXmlResult<float>& pbpt_scene_result;
    core::Scene&                             scene;
    resource::ResourceManager&               resources;
    const PbptImportOptions&                 options;
};

struct ExportGlobalContext {
    const core::Scene&                            scene;
    resource::ResourceManager&                    resources;
    const PbptCompatibleInfo&                     compatible_info;
    std::unordered_map<std::string, std::string>& material_name_by_reflectance;
};

}  // namespace rtr::framework::integration
