#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <glm/mat4x4.hpp>

namespace rtr::system::render {

struct ForwardSceneCameraData {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
};

struct ForwardSceneRenderable {
    std::uint64_t instance_id{0};
    std::string mesh_path{};
    std::string albedo_texture_path{};
    glm::mat4 model{1.0f};
    glm::mat4 normal{1.0f};
};

struct ForwardSceneView {
    ForwardSceneCameraData camera{};
    std::vector<ForwardSceneRenderable> renderables{};
};

} // namespace rtr::system::render
