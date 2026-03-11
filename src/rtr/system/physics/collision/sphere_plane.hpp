#pragma once

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision/base.hpp"
#include "rtr/system/physics/collision/plane_common.hpp"

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldSphere, WorldPlane> {
    static ContactResult generate(const WorldSphere& sphere, const WorldPlane& plane) {
        const auto plane_normal = detail::normalized_plane_normal(plane);
        const auto signed_distance = detail::signed_distance_to_plane(sphere.center, plane, plane_normal);
        if (signed_distance >= sphere.radius) {
            return ContactResult{};
        }

        return ContactResult{
            .point = sphere.center - plane_normal * sphere.radius,
            .normal = -plane_normal,
            .penetration = sphere.radius - signed_distance,
        };
    }
};

template <>
struct ContactPairTrait<WorldPlane, WorldSphere> {
    static ContactResult generate(const WorldPlane& plane, const WorldSphere& sphere) {
        auto result = ContactPairTrait<WorldSphere, WorldPlane>::generate(sphere, plane);
        result.normal = -result.normal;
        return result;
    }
};

}  // namespace rtr::system::physics
