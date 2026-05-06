#pragma once

#include <pbpt/math/math.h>
#include <vector>
#include <span>
#include <cmath>

namespace rtr::system::physics {

inline std::vector<pbpt::math::Vec3> recompute_vertex_normals(
    std::span<const pbpt::math::Vec3> positions,
    std::span<const uint32_t> indices
) {
    std::vector<pbpt::math::Vec3> normals(positions.size(), pbpt::math::Vec3{0.0f});

    for (size_t i = 0; i < indices.size(); i += 3) {
        uint32_t i0 = indices[i];
        uint32_t i1 = indices[i + 1];
        uint32_t i2 = indices[i + 2];

        const auto& v0 = positions[i0];
        const auto& v1 = positions[i1];
        const auto& v2 = positions[i2];

        pbpt::math::Vec3 face_normal = pbpt::math::cross(v1 - v0, v2 - v0);
        
        normals[i0] += face_normal;
        normals[i1] += face_normal;
        normals[i2] += face_normal;
    }

    for (auto& n : normals) {
        const auto len_sq = pbpt::math::dot(n, n);
        if (len_sq > 1e-12f) {
            n = n / std::sqrt(len_sq);
        } else {
            n = pbpt::math::Vec3{0.0f, 1.0f, 0.0f}; // Fallback
        }
    }

    return normals;
}

} // namespace rtr::system::physics
