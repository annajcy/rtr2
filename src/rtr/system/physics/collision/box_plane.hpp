#pragma once

#include <array>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision/base.hpp"
#include "rtr/system/physics/collision/plane_common.hpp"

namespace rtr::system::physics::detail {

inline std::array<pbpt::math::Vec3, 8> world_box_corners(const WorldBox& box) {
    const auto axis_x = pbpt::math::normalize(box.rotation * pbpt::math::Vec3{1.0f, 0.0f, 0.0f});
    const auto axis_y = pbpt::math::normalize(box.rotation * pbpt::math::Vec3{0.0f, 1.0f, 0.0f});
    const auto axis_z = pbpt::math::normalize(box.rotation * pbpt::math::Vec3{0.0f, 0.0f, 1.0f});

    std::array<pbpt::math::Vec3, 8> corners{};
    std::size_t index = 0;
    for (int sign_x : {-1, 1}) {
        for (int sign_y : {-1, 1}) {
            for (int sign_z : {-1, 1}) {
                corners[index++] =
                    box.center +
                    axis_x * (box.half_extents.x() * static_cast<pbpt::math::Float>(sign_x)) +
                    axis_y * (box.half_extents.y() * static_cast<pbpt::math::Float>(sign_y)) +
                    axis_z * (box.half_extents.z() * static_cast<pbpt::math::Float>(sign_z));
            }
        }
    }

    return corners;
}

}  // namespace rtr::system::physics::detail

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldBox, WorldPlane> {
    static ContactResult generate(const WorldBox& box, const WorldPlane& plane) {
        return detail::average_penetrating_points_against_plane(detail::world_box_corners(box), plane);
    }
};

template <>
struct ContactPairTrait<WorldPlane, WorldBox> {
    static ContactResult generate(const WorldPlane& plane, const WorldBox& box) {
        auto result = ContactPairTrait<WorldBox, WorldPlane>::generate(box, plane);
        result.normal = -result.normal;
        return result;
    }
};

}  // namespace rtr::system::physics
