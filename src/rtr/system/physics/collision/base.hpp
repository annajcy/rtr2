#pragma once

#include <pbpt/math/math.h>
#include "rtr/system/physics/type.hpp"


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

struct Contact {
    RigidBodyID        body_a{kInvalidRigidBodyId};
    RigidBodyID        body_b{kInvalidRigidBodyId};
    ColliderID         collider_a{kInvalidColliderId};
    ColliderID         collider_b{kInvalidColliderId};
    ContactResult      result{};

    bool is_valid() const { return result.is_valid(); }
};

struct SolverContact {
    RigidBodyID        body_a{kInvalidRigidBodyId};
    RigidBodyID        body_b{kInvalidRigidBodyId};
    ColliderID         collider_a{kInvalidColliderId};
    ColliderID         collider_b{kInvalidColliderId};
    pbpt::math::Vec3   point{0.0f};
    pbpt::math::Vec3   normal{0.0f, 1.0f, 0.0f};
    pbpt::math::Vec3   tangent{1.0f, 0.0f, 0.0f};
    pbpt::math::Float  penetration{0.0f};
    pbpt::math::Vec3   r_a{0.0f};
    pbpt::math::Vec3   r_b{0.0f};
    pbpt::math::Float  inv_mass_a{0.0f};
    pbpt::math::Float  inv_mass_b{0.0f};
    pbpt::math::Mat3   inv_inertia_a{pbpt::math::Mat3::zeros()};
    pbpt::math::Mat3   inv_inertia_b{pbpt::math::Mat3::zeros()};
    pbpt::math::Float  effective_mass_normal{0.0f};
    pbpt::math::Float  effective_mass_tangent{0.0f};
    pbpt::math::Float  restitution_bias{0.0f};
    pbpt::math::Float  normal_impulse_sum{0.0f};
    pbpt::math::Float  tangent_impulse_sum{0.0f};

    bool is_valid() const { return penetration > 0.0f; }
};

template <typename ColliderA, typename ColliderB>
struct ContactPairTrait {
    static ContactResult generate(const ColliderA& a, const ColliderB& b) {
        // 默认实现，返回无效接触
        return ContactResult{};
    }
};

}  // namespace rtr::system::physics
