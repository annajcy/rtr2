#pragma once

#include "rtr/system/physics/rigid_body/collision/collider_shape.hpp"
#include "rtr/system/physics/rigid_body/physics_ids.hpp"

namespace rtr::system::physics {

struct Collider {
    ColliderShape      shape{SphereShape{}};
    PhysicsTransform   local_transform{};
    RigidBodyID        rigid_body_id{kInvalidRigidBodyId};
};

}  // namespace rtr::system::physics
