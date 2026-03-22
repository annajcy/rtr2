# `obstacle_body.hpp`

`src/rtr/system/physics/ipc/model/obstacle_body.hpp` currently defines the thinnest possible placeholder obstacle type:

- `ObstacleBody`
  - owns only `IPCBodyInfo info{.type = IPCBodyType::Obstacle}`

This file exists for type-shape reasons rather than behavior. It lets the IPC model layer reserve a distinct obstacle category without pretending that obstacle kinematics, collision proxies, or assembly rules already exist.

The current intent is:

- keep the body-type vocabulary explicit
- avoid folding future obstacle handling into `TetBody`
- postpone the real obstacle representation until contact/barrier terms are implemented

So this file is intentionally a placeholder, not an incomplete solver implementation.
