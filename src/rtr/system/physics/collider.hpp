#pragma once

#include <variant>
#include "pbpt/geometry/bounds.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/math/basic/type_alias.hpp"
#include "pbpt/math/spatial/normal.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/type.hpp"
namespace rtr::system::physics {

enum class ColliderType {
    Box,
    Sphere,
    Capsule,
};

struct SphereShape {
    pbpt::math::Float radius;
};

struct PlaneShape {
    pbpt::math::Normal3 normal;
    pbpt::math::Float   offset;
};

using ColliderShape = std::variant<SphereShape, PlaneShape>;

struct Collider {
    ColliderShape           shape;
    pbpt::geometry::Bounds3 aabb;
    pbpt::geometry::Trans   local_transform;
};

}  // namespace rtr::system::physics
