# IPC Contact 总览

`src/rtr/system/physics/ipc/contact/` 是连接“求解器看到的全局状态”和“接触层需要的局部 primitive”的桥接层。

这一层必须单独存在，因为下面两层各自都不够：

- `core/` 知道全局 DOF 和 body 注册，但不知道表面 primitive
- `geometry/` 知道如何局部求一个 primitive pair 的距离，但不知道这些坐标从哪里来

`contact/` 的职责就是把这两端接起来：先构建统一 collision surface，再在这张表面上枚举候选 primitive pair。

## 当前模块

当前目录包含：

- `collision_mesh.hpp`：把 deformable tet boundary 和 static obstacle mesh 合并成统一的表面顶点 / 边 / 三角形索引空间
- `collision_candidates.hpp`：Day 2 的 brute-force broad phase，提供稳定顺序的 PT / EE 候选生成与过滤

## 职责边界

这一层刻意保持很窄：

- 它负责表面拓扑和 primitive ownership
- 它负责“一个 surface vertex 的当前位置应该如何读取”
- 它负责 broad-phase candidate 枚举
- 它不负责 barrier energy、CCD 根求解或稀疏全局装配

这样分层之后，整体数据流会比较干净：

```text
IPCSystem + ObstacleBody
    -> build_collision_mesh(...)
    -> 统一表面 primitive 索引空间
    -> build_collision_candidates(...)
    -> PT / EE candidate 列表
    -> 未来的 barrier / CCD wrapper
    -> geometry distance kernels
```

## 为什么要有 Collision Mesh

全局 `IPCState` 只存 deformable 顶点，静态 obstacle 顶点根本不在优化器状态里。

但接触层仍然需要一张统一的表面视图来回答：

- 这个 surface primitive 属于哪个 body
- 这个顶点是 deformable 还是 static
- 当前世界空间坐标该怎么取
- 后面如果要装配回全局 DOF，deformable 顶点映射到哪里

`CollisionMesh` 就是这张视图。它不是求解器状态的拷贝，而是一层“拓扑 + 映射”的桥。

## Day 2 范围

当前实现瞄准的是第一版可用的 IPC 接触切片：

- 来自 `IPCSystem` 的 deformable tet body
- 来自 `ObstacleBody` 的 static triangle mesh
- 双向 point-triangle candidates
- deformable side 和 obstacle side 之间的 edge-edge candidates

当前还不处理：

- self-contact
- deformable-vs-deformable contact
- obstacle-vs-obstacle contact
- BVH 或 spatial hash 加速

## 在系统中的位置

`contact/` 直接位于 `model/` 和 `core/` 之上，未来 barrier / CCD 之下。

- `model/mesh_tet_converter/tet_to_mesh.hpp` 提供 `TetSurfaceMapping`
- `core/ipc_system.hpp` 提供 body 列表和 deformable 状态
- `model/obstacle_body.hpp` 提供静态障碍物三角网格
- `contact/` 把这些来源合并进同一个 primitive index space
- 未来 barrier / CCD 会在这层产出的 candidates 上组装 `geometry::Distance` 的输入

正因为有了这层分离，geometry kernel 才能继续保持纯局部，而上层又能在 deformable/static 混合场景上工作。
