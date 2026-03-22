# `obstacle_body.hpp`

`src/rtr/system/physics/ipc/model/obstacle_body.hpp` 目前只定义了一个最薄的占位 obstacle 类型：

- `ObstacleBody`
  - 只持有 `IPCBodyInfo info{.type = IPCBodyType::Obstacle}`

这个文件存在的意义更多是“先把类型形状留出来”，而不是行为实现。它让 IPC model 层可以明确区分 obstacle 这一类对象，而不用假装 obstacle 的运动学、碰撞代理或装配规则已经完成。

当前保留它的目的主要有三点：

- 让 body 类型词汇保持明确
- 避免未来把 obstacle 逻辑硬塞进 `TetBody`
- 把真正的 obstacle 表示推迟到 contact / barrier 项实现之后再展开

所以这个文件现在是刻意保留的占位，而不是半成品求解器实现。
