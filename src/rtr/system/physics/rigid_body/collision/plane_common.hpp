#pragma once

#include <cstddef>

#include <pbpt/math/math.h>

#include "rtr/system/physics/rigid_body/collision/collider_shape.hpp"
#include "rtr/system/physics/rigid_body/collision/contact.hpp"

namespace rtr::system::physics::rb::detail::plane_common {

inline pbpt::math::Vec3 normalized_plane_normal(const WorldPlane& plane) {
    return pbpt::math::normalize(plane.normal);
}

inline pbpt::math::Float signed_distance_to_plane(const pbpt::math::Vec3& point,
                                                  const WorldPlane& plane,
                                                  const pbpt::math::Vec3& plane_normal) {
    return pbpt::math::dot(point - plane.point, plane_normal);
}

template <typename PointRange>
inline ContactResult average_penetrating_points_against_plane(const PointRange& points, const WorldPlane& plane) {
    const auto plane_normal = normalized_plane_normal(plane);

    pbpt::math::Vec3  accumulated_point{0.0f};
    pbpt::math::Float accumulated_penetration = 0.0f;
    std::size_t       hit_count = 0;

    for (const auto& point : points) {
        const auto signed_distance = signed_distance_to_plane(point, plane, plane_normal);
        if (signed_distance >= 0.0f) {
            continue;
        }

        accumulated_point += point;
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

}  // namespace rtr::system::physics::rb::detail::plane_common
