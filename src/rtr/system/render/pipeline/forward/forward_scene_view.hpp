#pragma once

#include <pbpt/math/math.h>

#include <cstdint>
#include <vector>

#include "rtr/rhi/mesh.hpp"

namespace rtr::system::render {

constexpr uint32_t kMaxPointLights = 4;

struct ForwardSceneCameraData {
    pbpt::math::Mat4 view{1.0f};
    pbpt::math::Mat4 proj{1.0f};
    pbpt::math::Vec3 world_pos{0.0f};
};

struct ForwardScenePointLight {
    pbpt::math::Vec3 position{0.0f};
    float            intensity{0.0f};
    pbpt::math::Vec3 color{0.0f};
    float            range{0.0f};
    float            specular_strength{0.0f};
    float            shininess{0.0f};
};

struct ForwardSceneRenderable {
    std::uint64_t    instance_id{0};
    rhi::Mesh&       mesh;
    pbpt::math::Vec4 base_color{1.0f, 1.0f, 1.0f, 1.0f};
    pbpt::math::Mat4 model{1.0f};
    pbpt::math::Mat4 normal{1.0f};
};

struct ForwardSceneView {
    ForwardSceneCameraData              camera{};
    std::vector<ForwardSceneRenderable> renderables{};
    std::vector<ForwardScenePointLight> point_lights{};
};

}  // namespace rtr::system::render
