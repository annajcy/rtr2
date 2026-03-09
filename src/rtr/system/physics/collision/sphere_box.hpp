#pragma once

#include <cmath>

#include "rtr/system/physics/collision/base.hpp"

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldSphere, WorldBox> {
    static ContactResult generate(const WorldSphere& sphere, const WorldBox& box) {
        // 1. 将球心转换到盒子局部空间
        const auto inv_rotation = box.rotation.inversed();
        const auto local_center = inv_rotation * (sphere.center - box.center);
        const auto min_extent   = -box.half_extents;
        const auto max_extent   = box.half_extents;

        // 2. 找到球心在盒子局部空间中最接近的点
        pbpt::math::Vec3 closest_local{
            pbpt::math::clamp(local_center.x(), min_extent.x(), max_extent.x()),
            pbpt::math::clamp(local_center.y(), min_extent.y(), max_extent.y()),
            pbpt::math::clamp(local_center.z(), min_extent.z(), max_extent.z()),
        };

        // 3. 计算球心与最近点的距离
        const auto delta_local = local_center - closest_local;
        const auto dist_sq     = pbpt::math::dot(delta_local, delta_local);

        ContactResult result{};
        if (dist_sq > sphere.radius * sphere.radius) {
            // 没有碰撞，返回最近点作为接触点，法线指向球心
            result.point       = box.center + box.rotation * closest_local;
            result.normal      = pbpt::math::normalize(sphere.center - result.point);
            result.penetration = 0.0f;
        } else {
            // 碰撞了，计算穿透深度和法线
            constexpr pbpt::math::Float kEpsilon = 1e-6f;
            if (dist_sq > kEpsilon) {
                const auto distance = std::sqrt(dist_sq);
                result.normal       = -(delta_local / distance);
                result.penetration  = sphere.radius - distance;
            } else {
                // 球心在盒子内部，无法确定法线方向，默认指向上方
                result.normal      = pbpt::math::Vec3{0.0f, 1.0f, 0.0f};
                result.penetration = sphere.radius;  // 穿透深度至少为一个球的半径
            }
            result.point = box.center + box.rotation * closest_local;
        }
        return result;
    }
};

template <>
struct ContactPairTrait<WorldBox, WorldSphere> {
    static ContactResult generate(const WorldBox& box, const WorldSphere& sphere) {
        auto result = ContactPairTrait<WorldSphere, WorldBox>::generate(sphere, box);
        result.normal = -result.normal;
        return result;
    }
};

}  // namespace rtr::system::physics
