#pragma once

#include <optional>
#include <variant>

#include <pbpt/math/math.h>

#include "rtr/system/physics/type.hpp"

namespace rtr::system::physics {

enum class ColliderType {
    Sphere,
    Box,
};

struct SphereShape {
    pbpt::math::Float radius{0.5f};
};

struct BoxShape {
    pbpt::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

using ColliderShape = std::variant<SphereShape, BoxShape>;

struct Collider {
    ColliderShape               shape{SphereShape{}};
    pbpt::math::Vec3            local_center{0.0f};
    pbpt::math::Quat            local_rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3            world_position{0.0f};
    pbpt::math::Quat            world_rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3            world_scale{1.0f, 1.0f, 1.0f};
    std::optional<RigidBodyID>  rigid_body_id{};
};

}  // namespace rtr::system::physics
