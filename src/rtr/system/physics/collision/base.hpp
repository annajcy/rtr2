#pragma once

#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"

namespace rtr::system::physics {

struct ContactResult {
    pbpt::math::Vec3   point{0.0f};
    pbpt::math::Vec3   normal{0.0f, 1.0f, 0.0f};
    pbpt::math::Float  penetration{0.0f};

    bool is_valid() const { return penetration > 0.0f; }
};

struct Contact {
    RigidBodyID        body_a{kInvalidRigidBodyId};
    RigidBodyID        body_b{kInvalidRigidBodyId};
    ColliderID         collider_a{kInvalidColliderId};
    ColliderID         collider_b{kInvalidColliderId};
    ContactResult      result{};

    bool is_valid() const { return result.is_valid(); }
};

template <typename ColliderA, typename ColliderB>
struct ContactPairTrait {
    static ContactResult generate(const ColliderA& a, const ColliderB& b) {
        // 默认实现，返回无效接触
        return ContactResult{};
    }
};

}  // namespace rtr::system::physics
