#pragma once

#include "rtr/system/physics/ipc/model/ipc_body.hpp"

namespace rtr::system::physics::ipc {

struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};
};

}  // namespace rtr::system::physics::ipc
