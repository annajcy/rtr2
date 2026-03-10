#pragma once

#include <variant>

#include <pbpt/math/math.h>

#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/spatial/vector.hpp"
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

struct WorldSphere {
    pbpt::math::Vec3  center{0.0f};
    pbpt::math::Float radius{0.5f};
};

struct WorldBox {
    pbpt::math::Vec3 center{0.0f};
    pbpt::math::Quat rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

using WorldCollider = std::variant<WorldSphere, WorldBox>;

template <typename FromType>
inline WorldCollider to_world_collider(const FromType& collider_shape, const PhysicsTransform& local_transform,
                                       const PhysicsTransform& world_transform);

template <>
inline WorldCollider to_world_collider(const BoxShape& box_shape, const PhysicsTransform& local_transform,
                                       const PhysicsTransform& world_transform) {
    pbpt::math::Vec3 scaled_local_pos = local_transform.position * world_transform.scale;
    pbpt::math::Vec3 center = world_transform.position + (world_transform.rotation * scaled_local_pos);
    pbpt::math::Quat rotation = pbpt::math::normalize(world_transform.rotation * local_transform.rotation);
    pbpt::math::Vec3 half_extents = box_shape.half_extents * local_transform.scale * world_transform.scale;

    return WorldBox{
        .center = center,
        .rotation = rotation,
        .half_extents = half_extents
    };
}

template <>
inline WorldCollider to_world_collider(const SphereShape& sphere_shape, const PhysicsTransform& local_transform,
                                       const PhysicsTransform& world_transform) {
    pbpt::math::Vec3 scaled_local_pos = local_transform.position * world_transform.scale;
    pbpt::math::Vec3 center = world_transform.position + (world_transform.rotation * scaled_local_pos);

    pbpt::math::Vec3 scale = local_transform.scale * world_transform.scale;
    pbpt::math::Float max_scale = scale.abs().max();

    return WorldSphere{
        .center = center,
        .radius = sphere_shape.radius * max_scale
    };
}

struct Collider {
    ColliderShape      shape{SphereShape{}};
    PhysicsTransform local_transform{};
    RigidBodyID        rigid_body_id{kInvalidRigidBodyId};
};

}  // namespace rtr::system::physics
