#pragma once

#include <cstdint>
#include <limits>

namespace rtr::system::physics {

using RigidBodyID = std::uint64_t;
using ColliderID  = std::uint64_t;

inline constexpr RigidBodyID kInvalidRigidBodyId = std::numeric_limits<RigidBodyID>::max();
inline constexpr ColliderID  kInvalidColliderId  = std::numeric_limits<ColliderID>::max();

}  // namespace rtr::system::physics
