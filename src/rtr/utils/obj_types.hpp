#pragma once

#include <cstdint>
#include <vector>

#include <glm/glm.hpp>

namespace rtr::utils {

struct ObjVertex {
    glm::vec3 position{0.0f};
    glm::vec2 uv{0.0f};
    glm::vec3 normal{0.0f};
};

struct ObjMeshData {
    std::vector<ObjVertex> vertices;
    std::vector<uint32_t> indices;
};

} // namespace rtr::utils
