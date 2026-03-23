#pragma once

#include <pbpt/math/basic/type_alias.hpp>

namespace rtr::system::physics::rb {

struct PhysicsMaterial {
    pbpt::math::Float restitution{0.0f};
    pbpt::math::Float friction{0.0f};
};

}  // namespace rtr::system::physics::rb
