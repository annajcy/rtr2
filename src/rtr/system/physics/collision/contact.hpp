#pragma once

#include <cmath>

#include <pbpt/math/math.h>

namespace rtr::system::physics {

struct ContactResult {
    pbpt::math::Vec3   point{0.0f};
    pbpt::math::Vec3   normal{0.0f, 1.0f, 0.0f};
    pbpt::math::Float  penetration{0.0f};

    bool is_valid() const { return penetration > 0.0f; }

    pbpt::math::Vec3 normalized_normal() const {
        constexpr pbpt::math::Float kEpsilon = 1e-6f;
        const auto length_sq = pbpt::math::dot(normal, normal);
        if (length_sq <= kEpsilon * kEpsilon) {
            return pbpt::math::Vec3{0.0f, 1.0f, 0.0f};
        }
        return normal / std::sqrt(length_sq);
    }
};

template <typename ColliderA, typename ColliderB>
struct ContactPairTrait {
    static ContactResult generate(const ColliderA& a, const ColliderB& b) {
        (void)a;
        (void)b;
        return ContactResult{};
    }
};

}  // namespace rtr::system::physics
