#pragma once

#include <variant>

#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"

namespace rtr::system::physics::ipc {

using TetMaterialVariant = std::variant<
    FixedCorotatedMaterial
>;

}  // namespace rtr::system::physics::ipc
