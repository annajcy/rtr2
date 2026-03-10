#pragma once

#include <cmath>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision/base.hpp"

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldMesh, WorldPlane> {
    static ContactResult generate(const WorldMesh& mesh, const WorldPlane& plane) {
        pbpt::math::Vec3 accumulated_point{0.0f};
        pbpt::math::Float accumulated_penetration = 0.0f;
        std::size_t hit_count = 0;

        const auto plane_normal = pbpt::math::normalize(plane.normal);
        for (const auto& vertex : mesh.vertices) {
            const auto signed_distance = pbpt::math::dot(vertex - plane.point, plane_normal);
            if (signed_distance >= 0.0f) {
                continue;
            }
            accumulated_point += vertex;
            accumulated_penetration += -signed_distance;
            ++hit_count;
        }

        if (hit_count == 0) {
            return ContactResult{};
        }

        return ContactResult{
            .point = accumulated_point / static_cast<pbpt::math::Float>(hit_count),
            .normal = -plane_normal,
            .penetration = accumulated_penetration / static_cast<pbpt::math::Float>(hit_count),
        };
    }
};

template <>
struct ContactPairTrait<WorldPlane, WorldMesh> {
    static ContactResult generate(const WorldPlane& plane, const WorldMesh& mesh) {
        auto result = ContactPairTrait<WorldMesh, WorldPlane>::generate(mesh, plane);
        result.normal = -result.normal;
        return result;
    }
};

}  // namespace rtr::system::physics
