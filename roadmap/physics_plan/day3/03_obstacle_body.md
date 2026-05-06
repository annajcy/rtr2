# Phase 3: ObstacleBody 升级

## 目标

把 `ObstacleBody` 从占位/半成品升级为可用的静态三角网格障碍物。

## 升级后的 ObstacleBody

```cpp
struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};

    // 世界空间顶点位置（不参与求解，不在全局 DOF 中）
    std::vector<Eigen::Vector3d> positions{};

    // 三角面片
    std::vector<std::array<std::size_t, 3>> triangles{};

    // 边（从三角形提取，去重）
    std::vector<std::array<std::size_t, 2>> edges{};

    std::size_t vertex_count() const;
    std::size_t triangle_count() const;

    void build_edges();
};
```

### 设计决策

- Obstacle 不纳入全局 DOF
- Obstacle 顶点存在自己的 `positions` 向量中
- barrier gradient/Hessian 只对 deformable body 的 DOF 有贡献
- collision mesh 通过 `global_dof_map` 中的 `nullopt` 标记 obstacle 顶点

## 内建生成器

### Ground Plane

```cpp
ObstacleBody generate_ground_plane(double size = 10.0, double y = 0.0);
```

生成水平矩形地面（2 个三角形，4 个顶点）：

```
(-size, y, -size) -- (size, y, -size)
       |                    |
(-size, y,  size) -- (size, y,  size)
```

### Sphere Obstacle（可选）

```cpp
ObstacleBody generate_sphere_obstacle(
    const Eigen::Vector3d& center,
    double radius,
    std::size_t subdivisions = 3
);
```

如果时间充裕再做。Day 3 优先保证 ground plane 可用。

## 与 CollisionMesh 的对接

Phase 1 完成后的 `CollisionMesh` 已经支持 obstacle 顶点。这里只需确保 `ObstacleBody` 的数据格式与 `build_collision_mesh()` 的输入兼容。

## IPCSystem 注册接口

```cpp
IPCBodyID create_obstacle_body(ObstacleBody body);
bool has_obstacle_body(IPCBodyID id) const;
bool remove_obstacle_body(IPCBodyID id);
const ObstacleBody& get_obstacle_body(IPCBodyID id) const;
```

Obstacle body 的添加/删除触发 collision mesh 重建，但不触发 DOF 重建。

## 测试

- `obstacle_body_test.cpp`：ground plane 生成正确、顶点数、三角形数、边数
- 边提取去重正确

## 输出

修改：
- `src/rtr/system/physics/ipc/model/obstacle_body.hpp`

新增测试：
- `test/system/physics/ipc/model/obstacle_body_test.cpp`（如果尚不存在）
