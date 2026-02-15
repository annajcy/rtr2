#pragma once

#include <pbpt/math/math.h>

#include <cstdint>
#include <vector>


namespace rtr::utils {

struct ObjVertex {
    pbpt::math::vec3 position{0.0f};
    pbpt::math::vec2 uv{0.0f};
    pbpt::math::vec3 normal{0.0f};
};

struct ObjMeshData {
    std::vector<ObjVertex> vertices;
    std::vector<uint32_t> indices;
};

} // namespace rtr::utils
