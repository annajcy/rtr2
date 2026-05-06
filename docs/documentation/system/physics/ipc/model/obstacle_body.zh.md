# `obstacle_body.hpp`

`src/rtr/system/physics/ipc/model/obstacle_body.hpp` 定义了当前 IPC 接触管线使用的静态障碍物表示。

## 当前数据形状

`ObstacleBody` 当前包含：

- `IPCBodyInfo info{.type = IPCBodyType::Obstacle}`
- `positions`：障碍物的世界空间顶点
- `triangles`：基于 `positions` 的三角形拓扑
- `edges`：从三角形提取并去重后的无向边

这个 obstacle 刻意放在全局优化状态之外：

- obstacle 顶点不会追加进 `IPCState.x`
- `info.dof_offset` 对求解器装配没有实际作用
- 接触层直接从 `positions` 读取 obstacle 坐标

这正符合当前 Day 2 的范围：obstacle 是只用于接触预处理的静态几何。

## 校验与边提取

这个文件还提供了两个行为辅助：

- `validate()`：检查每个三角形索引都落在 `positions` 范围内
- `build_edges()`：从三角面提取 `(i, j)` 边，对端点做 `min/max` 规范化，然后排序去重

这样 `ObstacleBody` 就能被 `contact/collision_mesh.hpp` 直接消费，而不用让 contact 层重复实现一遍障碍物拓扑清理逻辑。

## 为什么 Obstacle 放在 `model/`

`ObstacleBody` 属于 `model/`，因为它表达的是几何和 ownership 元数据，而不是求解器状态：

- `core/` 负责 deformable 未知量的全局 DOF 向量
- `model/` 负责 body-local 的几何和拓扑
- `contact/` 再把 `ObstacleBody` 和 tet boundary 合并成统一 collision surface

把 obstacle mesh 留在这里，可以避免静态几何污染优化器视角下的 `IPCState` 布局。

## 当前边界

这个文件目前仍然刻意保持最小：

- 已支持：静态三角网格、基础校验、边提取
- 还不支持：kinematic obstacle 更新、刚体变换、法线、UV 或 contact cache

这已经足够支撑第一版 collision mesh 和 broad phase 管线，同时不会过早把未来 obstacle API 固死。
