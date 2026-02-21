#pragma once

#include <string>
#include <unordered_map>

#include "pbpt/serde/scene_loader.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::core {
class Scene;
}

namespace rtr::framework::integration {

struct LoadOptions;
struct CompatibleInfo;

struct ImportGlobalContext {
    const ::pbpt::serde::PbptXmlResult<float>& pbpt_scene_result;
    core::Scene&                              scene;
    resource::ResourceManager&                resources;
    const LoadOptions&                        options;
};

struct ExportGlobalContext {
    const core::Scene&                            scene;
    resource::ResourceManager&                    resources;
    const CompatibleInfo&                         compatible_info;
    std::unordered_map<std::string, std::string>& material_name_by_reflectance;
};

}  // namespace rtr::framework::integration
