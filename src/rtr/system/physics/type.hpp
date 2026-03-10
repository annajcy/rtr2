#pragma once

#include <cstdint>
#include <limits>

#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/matrix/matrix.hpp"
#include "pbpt/math/matrix/matrix_transform.hpp"
#include "pbpt/math/spatial/vector.hpp"

namespace rtr::system::physics {

using RigidBodyID = std::uint64_t;
using ColliderID  = std::uint64_t;

inline constexpr RigidBodyID kInvalidRigidBodyId = std::numeric_limits<RigidBodyID>::max();
inline constexpr ColliderID  kInvalidColliderId  = std::numeric_limits<ColliderID>::max();

struct PhysicsTransform {
    pbpt::math::Vec3  position{0.0f};
    pbpt::math::Quat rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 scale{1.0f};

    pbpt::math::Mat4 to_transform() const {
        return pbpt::math::compose_trs(position, rotation, scale);
    }
};

}  // namespace rtr::system::physics
