# `collision_mesh.hpp`

`src/rtr/system/physics/ipc/contact/collision_mesh.hpp` 定义了 IPC 接触层使用的统一表面表示。

它的任务是把不同来源的场景数据整理成同一个 primitive 索引空间：

- deformable tet boundary 来自 `TetSurfaceMapping`
- obstacle triangle 来自 `ObstacleBody`
- 最终统一落成 `CollisionVertex`、`CollisionTriangle` 和 `CollisionEdge`

## 为什么需要这一层

未来 barrier 和 CCD 需要的不只是坐标本身，还包括：

- primitive 归属于哪个 body
- 稳定的 surface vertex 索引
- 当前坐标应该如何读取
- 如果顶点是 deformable，怎样回到全局 DOF 索引

这些都不应该放进 `geometry/`，因为 geometry kernel 刻意保持无拓扑；也不应该塞进 `IPCState`，因为静态 obstacle 顶点根本不在优化变量里。

所以 `CollisionMesh` 是一个桥接对象，而不是求解器状态容器。

## 数据模型

这个文件定义了三个 primitive 记录和一个聚合 mesh：

```cpp
enum class CollisionVertexKind {
    Deformable,
    Obstacle,
};

struct CollisionVertex {
    IPCBodyID body_id;
    CollisionVertexKind kind;
    std::size_t body_vertex_index;
    std::optional<std::size_t> global_vertex_index;
    Eigen::Vector3d static_position;
};
```

几个关键点：

- `body_id` 记录 primitive ownership
- `body_vertex_index` 记住它在源 body 内的局部顶点编号
- `global_vertex_index` 只对 deformable 顶点有效
- `static_position` 只对 obstacle 顶点有效

聚合后的 `CollisionMesh` 保存：

- `vertices`
- `triangles`
- `edges`
- 每种 primitive 的 deformable / obstacle 预分类索引列表

这批缓存索引很重要，因为 broad phase 的组织方式本来就是按 primitive 类别来循环，而不是每次重新扫一遍 body type。

## 顶点位置读取协议

`read_collision_vertex_position(...)` 是统一读取 surface vertex 位置的唯一规则。

对 deformable 顶点：

$$
x_i = \texttt{state.position(global\_vertex\_index)}
$$

对 obstacle 顶点：

$$
x_i = \texttt{static\_position}
$$

这是后续 barrier / CCD wrapper 能保持简洁的关键契约。它们不需要再关心一个顶点究竟来自优化器状态还是静态网格。

## Mesh 构建流程

主入口是：

```cpp
CollisionMesh build_collision_mesh(const IPCSystem& system);
```

构建分三步进行。

### 1. 追加 Tet Surface 顶点和三角形

对每个启用的 tet body：

1. 调用 `build_tet_surface_mapping(body)`
2. 对每个 surface vertex id 分配一个 `CollisionVertex`
3. 它的 global vertex index 按下面公式计算：

$$
\texttt{body.info.dof\_offset / 3 + local\_vertex\_id}
$$

4. 把 `surface_indices` 重映射成 collision-mesh-local 编号下的 `CollisionTriangle`

实现里会先建立一张“tet-local surface vertex id -> collision-mesh vertex id”的局部表，这样三角形追加时就不会把原始索引方案泄漏到更高层。

### 2. 追加 Obstacle 顶点和三角形

对每个启用的 obstacle body：

1. 为 `positions[i]` 逐个追加 obstacle `CollisionVertex`
2. 设置 `global_vertex_index = nullopt`
3. 拷贝 `static_position = positions[i]`
4. 以一个 surface-vertex 偏移量追加 obstacle triangles

Day 2 把 obstacle 当成静态世界空间几何，因此这里直接保留 world-space 位置。

### 3. 提取并去重表面边

Tet surface mapping 不直接带边，所以这个文件从每个三角形提取三条边：

- `(v0, v1)`
- `(v1, v2)`
- `(v0, v2)`

每条边的 key 记成：

```cpp
(body_id, min(a, b), max(a, b))
```

最后用排序加去重得到稳定的 edge set。

把 `body_id` 放进 key 是有意的。它既保留了 ownership，也避免“恰好全局编号撞在一起”把不同 body 的边错误混成同一条。

## Primitive 分类

当 vertices、triangles 和 edges 都构建好之后，`classify_primitives(...)` 会填充：

- `deformable_vertex_indices`
- `obstacle_vertex_indices`
- `deformable_triangle_indices`
- `obstacle_triangle_indices`
- `deformable_edge_indices`
- `obstacle_edge_indices`

当前实现通过 triangle 或 edge 的第一个顶点来判断它属于哪一侧。因为现在每个 primitive 都保证只来自同一个 body、同一类 side，所以这种做法是安全且便宜的。

## 在系统中的作用

`CollisionMesh` 是后续 IPC 接触层的“拓扑 + 映射”输入：

- broad phase 消费它的 primitive 列表
- barrier wrapper 会从它读取 PT / EE 的局部坐标
- CCD wrapper 会复用同一套拓扑，只是换试探状态下的坐标

这里最重要的设计点是：`CollisionMesh` 不缓存 deformable 的当前位置。这样它就不会绑定到某次 Newton iterate，而能在不同试探状态之间复用。
