#pragma once

#include <cstdint>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec4.hpp>

#include "rtr/resource/resource_types.hpp"

namespace rtr::system::render {

struct ForwardSceneCameraData {
    glm::mat4 view{1.0f};
    glm::mat4 proj{1.0f};
};

struct ForwardSceneRenderable {
    std::uint64_t instance_id{0};
    resource::MeshHandle mesh{};
    glm::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
    glm::mat4 model{1.0f};
    glm::mat4 normal{1.0f};
};

struct ForwardSceneView {
    ForwardSceneCameraData camera{};
    std::vector<ForwardSceneRenderable> renderables{};
};

} // namespace rtr::system::render
