#pragma once

#include <variant>
#include <vector>

#include <pbpt/math/math.h>

#include "pbpt/math/complex/quaternion.hpp"
#include "pbpt/math/spatial/vector.hpp"
#include "rtr/system/physics/type.hpp"

namespace rtr::system::physics {

enum class ColliderType {
    Sphere,
    Box,
    Plane,
    Mesh,
};

struct SphereShape {
    pbpt::math::Float radius{0.5f};
};

struct BoxShape {
    pbpt::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

struct PlaneShape {
    pbpt::math::Vec3 normal_local{0.0f, 1.0f, 0.0f};
};

struct MeshShape {
    std::vector<pbpt::math::Vec3> local_vertices{};
};

using ColliderShape = std::variant<SphereShape, BoxShape, PlaneShape, MeshShape>;

struct WorldSphere {
    pbpt::math::Vec3  center{0.0f};
    pbpt::math::Float radius{0.5f};
};

struct WorldBox {
    pbpt::math::Vec3 center{0.0f};
    pbpt::math::Quat rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

struct WorldPlane {
    pbpt::math::Vec3 point{0.0f};
    pbpt::math::Vec3 normal{0.0f, 1.0f, 0.0f};
};

struct WorldMesh {
    std::vector<pbpt::math::Vec3> vertices{};
};

using WorldCollider = std::variant<WorldSphere, WorldBox, WorldPlane, WorldMesh>;

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

template <>
inline WorldCollider to_world_collider(const PlaneShape& plane_shape, const PhysicsTransform& local_transform,
                                       const PhysicsTransform& world_transform) {
    const pbpt::math::Vec3 scaled_local_pos = local_transform.position * world_transform.scale;
    const pbpt::math::Vec3 point = world_transform.position + (world_transform.rotation * scaled_local_pos);
    const pbpt::math::Quat rotation = pbpt::math::normalize(world_transform.rotation * local_transform.rotation);

    return WorldPlane{
        .point = point,
        .normal = pbpt::math::normalize(rotation * plane_shape.normal_local),
    };
}

template <>
inline WorldCollider to_world_collider(const MeshShape& mesh_shape, const PhysicsTransform& local_transform,
                                       const PhysicsTransform& world_transform) {
    WorldMesh world_mesh{};
    world_mesh.vertices.reserve(mesh_shape.local_vertices.size());
    for (const auto& vertex : mesh_shape.local_vertices) {
        const pbpt::math::Vec3 local_scaled = vertex * local_transform.scale;
        const pbpt::math::Vec3 body_local = local_transform.position + (local_transform.rotation * local_scaled);
        const pbpt::math::Vec3 world_scaled = body_local * world_transform.scale;
        world_mesh.vertices.push_back(world_transform.position + (world_transform.rotation * world_scaled));
    }
    return world_mesh;
}

struct Collider {
    ColliderShape      shape{SphereShape{}};
    PhysicsTransform local_transform{};
    RigidBodyID        rigid_body_id{kInvalidRigidBodyId};
};

}  // namespace rtr::system::physics
