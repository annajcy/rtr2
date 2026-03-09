#pragma once

#include <cmath>
#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"

namespace rtr::system::physics {

struct Contact {
    RigidBodyID        body_a{kInvalidRigidBodyId};
    RigidBodyID        body_b{kInvalidRigidBodyId};
    ColliderID         collider_a{kInvalidColliderId};
    ColliderID         collider_b{kInvalidColliderId};
    pbpt::math::Vec3   point{0.0f};
    pbpt::math::Vec3   normal{0.0f, 1.0f, 0.0f};
    pbpt::math::Float  penetration{0.0f};
};

namespace detail {

inline pbpt::math::Vec3 abs_vec3(const pbpt::math::Vec3& value) {
    return pbpt::math::Vec3{std::abs(value.x()), std::abs(value.y()), std::abs(value.z())};
}

inline pbpt::math::Vec3 hadamard(const pbpt::math::Vec3& lhs, const pbpt::math::Vec3& rhs) {
    return pbpt::math::Vec3{lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z()};
}

inline pbpt::math::Float max_component(const pbpt::math::Vec3& value) {
    return std::max(value.x(), std::max(value.y(), value.z()));
}

struct WorldSphere {
    pbpt::math::Vec3  center{0.0f};
    pbpt::math::Float radius{0.5f};
};

struct WorldBox {
    pbpt::math::Vec3 center{0.0f};
    pbpt::math::Quat rotation{pbpt::math::Quat::identity()};
    pbpt::math::Vec3 half_extents{0.5f, 0.5f, 0.5f};
};

inline WorldSphere world_sphere(const Collider& collider) {
    const auto* sphere = std::get_if<SphereShape>(&collider.shape);
    const auto  scale  = abs_vec3(collider.world_scale);
    return WorldSphere{
        .center = collider.world_position,
        .radius = sphere != nullptr ? sphere->radius * max_component(scale) : 0.0f,
    };
}

inline WorldBox world_box(const Collider& collider) {
    const auto* box   = std::get_if<BoxShape>(&collider.shape);
    const auto  scale = abs_vec3(collider.world_scale);
    return WorldBox{
        .center       = collider.world_position,
        .rotation     = pbpt::math::normalize(collider.world_rotation),
        .half_extents = box != nullptr ? hadamard(box->half_extents, scale) : pbpt::math::Vec3{0.0f},
    };
}

}  // namespace detail

inline std::optional<Contact> collide_sphere_sphere(ColliderID a_id, const Collider& a, ColliderID b_id,
                                                    const Collider& b) {
    const auto sphere_a   = detail::world_sphere(a);
    const auto sphere_b   = detail::world_sphere(b);
    const auto delta      = sphere_b.center - sphere_a.center;
    const auto dist_sq    = pbpt::math::dot(delta, delta);
    const auto radius_sum = sphere_a.radius + sphere_b.radius;
    if (dist_sq > radius_sum * radius_sum) {
        return std::nullopt;
    }

    constexpr pbpt::math::Float kEpsilon = 1e-6f;
    pbpt::math::Vec3             normal{1.0f, 0.0f, 0.0f};
    pbpt::math::Float            distance = 0.0f;
    if (dist_sq > kEpsilon) {
        distance = std::sqrt(dist_sq);
        normal   = delta / distance;
    }

    return Contact{
        .body_a      = a.rigid_body_id,
        .body_b      = b.rigid_body_id,
        .collider_a  = a_id,
        .collider_b  = b_id,
        .point       = sphere_a.center + normal * sphere_a.radius,
        .normal      = normal,
        .penetration = radius_sum - distance,
    };
}

inline std::optional<Contact> collide_sphere_box(ColliderID sphere_id, const Collider& sphere_collider,
                                                 ColliderID box_id, const Collider& box_collider) {
    const auto sphere = detail::world_sphere(sphere_collider);
    const auto box    = detail::world_box(box_collider);

    const auto inv_rotation = box.rotation.inversed();
    const auto local_center = inv_rotation * (sphere.center - box.center);
    const auto min_extent   = -box.half_extents;
    const auto max_extent   = box.half_extents;

    pbpt::math::Vec3 closest_local{
        pbpt::math::clamp(local_center.x(), min_extent.x(), max_extent.x()),
        pbpt::math::clamp(local_center.y(), min_extent.y(), max_extent.y()),
        pbpt::math::clamp(local_center.z(), min_extent.z(), max_extent.z()),
    };

    const auto delta_local = local_center - closest_local;
    const auto dist_sq     = pbpt::math::dot(delta_local, delta_local);
    if (dist_sq > sphere.radius * sphere.radius) {
        return std::nullopt;
    }

    constexpr pbpt::math::Float kEpsilon = 1e-6f;
    pbpt::math::Vec3             normal_local{0.0f, -1.0f, 0.0f};
    pbpt::math::Float            penetration = 0.0f;

    if (dist_sq > kEpsilon) {
        const auto distance = std::sqrt(dist_sq);
        normal_local        = -(delta_local / distance);
        penetration         = sphere.radius - distance;
    } else {
        const auto distance_to_face = box.half_extents - detail::abs_vec3(local_center);

        int axis = 0;
        if (distance_to_face.y() < distance_to_face.x()) {
            axis = 1;
        }
        if (distance_to_face.z() < distance_to_face[axis]) {
            axis = 2;
        }

        const pbpt::math::Float sign = local_center[axis] >= 0.0f ? 1.0f : -1.0f;
        normal_local                 = pbpt::math::Vec3{0.0f};
        normal_local[axis]           = -sign;
        closest_local                = local_center;
        closest_local[axis]          = sign * box.half_extents[axis];
        penetration                  = sphere.radius + distance_to_face[axis];
    }

    return Contact{
        .body_a      = sphere_collider.rigid_body_id,
        .body_b      = box_collider.rigid_body_id,
        .collider_a  = sphere_id,
        .collider_b  = box_id,
        .point       = box.center + box.rotation * closest_local,
        .normal      = pbpt::math::normalize(box.rotation * normal_local),
        .penetration = penetration,
    };
}

}  // namespace rtr::system::physics
