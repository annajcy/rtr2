#pragma once

#include <pbpt/math/math.h>

#include <cstdint>
#include <vector>


#include "rtr/resource/resource_types.hpp"

namespace rtr::system::render {

struct ForwardSceneCameraData {
    pbpt::math::mat4 view{1.0f};
    pbpt::math::mat4 proj{1.0f};
};

struct ForwardSceneRenderable {
    std::uint64_t instance_id{0};
    resource::MeshHandle mesh{};
    pbpt::math::vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
    pbpt::math::mat4 model{1.0f};
    pbpt::math::mat4 normal{1.0f};
};

struct ForwardSceneView {
    ForwardSceneCameraData camera{};
    std::vector<ForwardSceneRenderable> renderables{};
};

} // namespace rtr::system::render
