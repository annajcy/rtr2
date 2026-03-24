# Phase 1: 完成 Collision Mesh + Candidates

## 目标

把 Day 2 已创建但未完成的 `CollisionMesh` 和 `CollisionCandidates` 闭环，使其能同时处理 tet body 表面和 obstacle 表面。

## 当前状态

`collision_mesh.hpp` 和 `collision_candidates.hpp` 已存在但尚未完成。需要对照 Day 2 Phase 2 计划完善以下功能。

## CollisionMesh 必须完成的功能

### 1. 合并 tet 表面 + obstacle 表面

从所有注册的 tet body 的 `TetSurfaceMapping` 和所有 obstacle body 的三角网格中，构建一个统一的接触表面：

- 全局表面顶点索引
- 全局表面三角形索引
- 全局边索引（从三角形拓扑提取，去重）

### 2. 顶点读取路径

每个表面顶点需要知道如何读取其位置：

- **deformable 顶点**：从 `IPCState.x` 的对应全局 DOF 索引读取
- **obstacle 顶点**：从 `ObstacleBody.positions` 直接读取

推荐接口：

```cpp
Eigen::Vector3d vertex_position(
    std::size_t surface_vertex_idx,
    const Eigen::VectorXd& dof_x
) const;
```

### 3. DOF 映射

每个表面顶点需要记录其对应的全局 DOF 索引（如果是 deformable）或标记为 obstacle（不在 DOF 中）：

```cpp
// global_dof_index[surface_vertex_idx] = 全局 DOF 起始索引（3 个连续分量）
// 对 obstacle 顶点为 std::nullopt
std::vector<std::optional<Eigen::Index>> global_dof_index{};
```

### 4. Body 归属

每个表面元素需要记录属于哪个 body，用于候选过滤（同体非邻接过滤）。

### 5. 边提取

从所有表面三角形中提取边并去重。边用于 edge-edge 候选对生成。

## CollisionCandidates 必须完成的功能

### 1. PT 候选生成

遍历所有 (表面顶点, 表面三角形) 对，生成 PT 候选。

### 2. EE 候选生成

遍历所有 (边, 边) 对，生成 EE 候选。

### 3. 候选过滤

- **共享顶点过滤**：PT 中点是三角形的顶点 → 跳过
- **共享边过滤**：EE 中两条边共享顶点 → 跳过
- **同体邻接过滤**：同一 body 的拓扑相邻原语 → 跳过

### 4. 距离阈值过滤（可选优化）

只保留当前距离 < 某个阈值（如 `inflation * dhat`）的候选对，减少后续 barrier 计算量。Day 3 先不做，brute force 全部保留。

## 测试

- `collision_mesh_test.cpp`：验证顶点/三角形/边数量正确，obstacle 顶点标记正确
- `collision_candidates_test.cpp`：验证候选数量、过滤逻辑正确

## 输出

完善后的：
- `src/rtr/system/physics/ipc/contact/collision_mesh.hpp`
- `src/rtr/system/physics/ipc/contact/collision_candidates.hpp`
- `test/system/physics/ipc/contact/collision_mesh_test.cpp`
- `test/system/physics/ipc/contact/collision_candidates_test.cpp`
