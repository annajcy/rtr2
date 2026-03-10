#pragma once

#include <cmath>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision/base.hpp"

namespace rtr::system::physics {

template <>
struct ContactPairTrait<WorldSphere, WorldSphere> {
    static ContactResult generate(const WorldSphere& a, const WorldSphere& b) {
        const auto delta      = b.center - a.center;
        const auto dist_sq    = pbpt::math::dot(delta, delta);
        const auto radius_sum = a.radius + b.radius;

        constexpr pbpt::math::Float kEpsilon = 1e-6f;
        ContactResult result{};
        if (dist_sq < radius_sum * radius_sum) {
            if (dist_sq > kEpsilon) {
                const auto distance = std::sqrt(dist_sq);
                result.point        = a.center + delta * (a.radius / distance);
                result.normal       = delta / distance;
                result.penetration  = radius_sum - distance;
            } else {
                // 完全重叠时，无法确定法线方向，默认指向上方
                result.point       = a.center;
                result.normal      = pbpt::math::Vec3{0.0f, 1.0f, 0.0f};
                result.penetration = a.radius;  // 穿透深度至少为一个球的半径
            }
        }
        return result;
    }
};

}  // namespace rtr::system::physics
