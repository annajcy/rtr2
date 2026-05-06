# Phase 4: ObstacleBody

## 目标

把 Day 1 的 `ObstacleBody` 占位结构升级为可用的静态三角网格障碍物，使其可以参与接触检测。

## 当前状态

Day 1 的 `obstacle_body.hpp` 只是一个空占位：

```cpp
struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};
};
```

## 升级后的 ObstacleBody

```cpp
struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};

    // 顶点位置（world space，obstacle 不参与求解，位置不随 DOF 变化）
    std::vector<Eigen::Vector3d> positions{};

    // 三角面片（引用 positions 的索引）
    std::vector<std::array<std::size_t, 3>> triangles{};

    // 边（从三角形提取，去重）
    std::vector<std::array<std::size_t, 2>> edges{};

    std::size_t vertex_count() const { return positions.size(); }
    std::size_t triangle_count() const { return triangles.size(); }

    // 从三角形拓扑提取边并去重
    void build_edges();
};
```

### 关键设计决策

**Obstacle 不纳入全局 DOF**。

rigid body 类的障碍物不应该被 Newton solver 当做自由度来优化。它们的位置是给定的（static）或按时间函数更新的（kinematic）。

所以：
- `ObstacleBody` 的顶点存在自己的 `positions` 向量中，不在 `IPCState.x` 里
- 计算 PT/EE 距离时，obstacle 顶点直接从 `positions` 读取
- barrier gradient/Hessian 只对 deformable body 的 DOF 有贡献（obstacle 侧梯度为零）

---

## 常用 Obstacle 生成器

Day 2 demo 需要的最小障碍物：

### Ground Plane

```cpp
ObstacleBody generate_ground_plane(double size = 10.0, double y = 0.0);
```

生成一个水平矩形面片（两个三角形），位于 $y = y_0$：

```
(-size, y, -size) -- (size, y, -size)
       |                    |
(-size, y,  size) -- (size, y,  size)
```

两个三角形，4 个顶点，2 条对角边 + 4 条外边 = 5 条边。

### Sphere Obstacle（可选）

```cpp
ObstacleBody generate_sphere_obstacle(
    const Eigen::Vector3d& center,
    double radius,
    std::size_t subdivisions = 3
);
```

UV 球或 icosphere 的三角化。

---

## IPCSystem 集成

### 注册 API

类似 tet body，给 `IPCSystem` 加 obstacle 注册接口：

```cpp
IPCBodyID create_obstacle_body(ObstacleBody body);
bool has_obstacle_body(IPCBodyID id) const;
bool remove_obstacle_body(IPCBodyID id);
const ObstacleBody& get_obstacle_body(IPCBodyID id) const;
```

### Obstacle 不影响 rebuild_runtime_state

obstacle body 不占 DOF，不参与 `m_state.resize()`。它们只被 collision mesh builder 和 barrier 装配引用。

### collision mesh builder 中的 obstacle 处理

`build_collision_mesh()` 需要为 obstacle body 的表面三角形分配"虚拟"表面顶点索引。这些顶点不在全局 DOF 中，所以 `vertex_map` 需要一种方式区分：

方案 A：obstacle 顶点用特殊的 sentinel 值标记，barrier 装配时特殊处理。

方案 B（推荐）：collision mesh 存两份坐标读取路径——deformable 顶点从 `state.x` 读，obstacle 顶点从 `ObstacleBody.positions` 读。

推荐 **方案 B**。具体做法：

```cpp
struct CollisionMesh {
    // ... 原有字段 ...

    // obstacle 顶点的位置（直接存储，不在全局 DOF 中）
    // obstacle_vertex_positions[surface_idx] 只在该顶点属于 obstacle 时有效
    std::vector<Eigen::Vector3d> obstacle_vertex_positions{};

    // 标记该表面顶点是否是 obstacle 顶点
    std::vector<bool> is_obstacle_vertex{};

    // 读取表面顶点位置的统一接口
    Eigen::Vector3d vertex_position(
        std::size_t surface_idx,
        const Eigen::VectorXd& dof_x
    ) const;
};
```

这样 barrier 装配代码统一调 `collision_mesh.vertex_position(i, state.x)` 就行，不需要区分 obstacle 和 deformable。

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/model/obstacle_body.hpp` | 升级后的 ObstacleBody + generators |
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | 更新：支持 obstacle 顶点 |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/model/obstacle_body_test.cpp` | ground plane 生成正确、边提取、edge count |
