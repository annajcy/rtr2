#pragma once

#include <pbpt/math/math.h>
#include <cstdint>
#include <vector>

namespace rtr::system::physics {

struct DeformableMeshState {
    std::vector<pbpt::math::Vec3> positions;
    std::vector<pbpt::math::Vec3> velocities;
    std::vector<pbpt::math::Float> masses;
    std::vector<uint32_t> indices; // triangle index buffer (surface)

    uint32_t vertex_count() const {
        return static_cast<uint32_t>(positions.size());
    }
};

} // namespace rtr::system::physics
