# Phase 2: Collision Mesh + Broad Phase

## 目标

构建统一的碰撞几何表示（collision mesh），并实现最小可用的 broad phase 候选对生成。

## 为什么需要 collision mesh

IPCSystem 内部的 DOF 是全局展平的 `Eigen::VectorXd`，但接触检测需要的是**表面三角形和边**的拓扑信息。collision mesh 是这两者之间的桥接层。

Day 1 已有的 `TetSurfaceResult`（`extract_tet_surface`）提供了单个 body 的表面三角形和表面顶点映射。collision mesh 在此基础上：

1. 合并所有 body 的表面几何（tet body 表面 + obstacle body 三角形）
2. 提取边连接关系
3. 标记每个原语（顶点/边/三角形）属于哪个 body

---

## Collision Mesh 数据结构

```cpp
struct CollisionMesh {
    // 表面顶点在全局 DOF 中的索引（surface_vertex_id → global_vertex_id）
    std::vector<std::size_t> vertex_map{};

    // 表面三角形（local surface index）
    std::vector<std::array<std::size_t, 3>> triangles{};

    // 表面边（从三角形拓扑提取，去重）
    std::vector<std::array<std::size_t, 2>> edges{};

    // 每个表面顶点属于哪个 body（IPCBodyID）
    std::vector<IPCBodyID> vertex_body{};

    // 每条边属于哪个 body
    std::vector<IPCBodyID> edge_body{};

    // 每个三角形属于哪个 body
    std::vector<IPCBodyID> triangle_body{};

    std::size_t vertex_count() const { return vertex_map.size(); }
    std::size_t triangle_count() const { return triangles.size(); }
    std::size_t edge_count() const { return edges.size(); }
};
```

### vertex_map 的作用

collision mesh 的顶点索引是"表面局部索引"（0, 1, 2, ...），但距离函数需要读全局 DOF 中的坐标。`vertex_map[surface_idx]` 给出全局 DOF 的 vertex index，用于 `state.x.segment<3>(3 * vertex_map[i])` 定位。

### 边提取

从三角形列表中提取边，去重：

```
对每个三角形 (v0, v1, v2):
    添加边 (min(v0,v1), max(v0,v1))
    添加边 (min(v1,v2), max(v1,v2))
    添加边 (min(v0,v2), max(v0,v2))
用 set 或 sort+unique 去重
```

### 构建流程

```cpp
CollisionMesh build_collision_mesh(
    const IPCSystem& system  // 读取所有 body 的表面信息
);
```

内部：
1. 遍历所有 tet body：用 `extract_tet_surface()` 获取表面三角形，`surface_vertex_ids` 映射到全局
2. 遍历所有 obstacle body：直接取其三角面片
3. 合并到统一的 collision mesh，记录 body 归属

---

## Broad Phase

### Day 2 策略：先用 brute force

Day 2 的 broad phase 只要求**正确**，不要求快。先实现 $O(n^2)$ 的暴力枚举：

- 对所有 (顶点, 三角形) 对生成 PT 候选
- 对所有 (边, 边) 对生成 EE 候选

这在小规模场景（< 1000 表面顶点）下完全够用。

### 候选对过滤规则

生成候选后，必须过滤掉以下非法候选：

#### 1. 共享顶点过滤

如果 PT 对中的 point 是三角形的某个顶点，距离恒为 0 且无物理意义。

```cpp
// PT 候选：point_idx 不能是 tri 的任一顶点
if (point_idx == tri[0] || point_idx == tri[1] || point_idx == tri[2]) {
    skip;
}
```

#### 2. 共享边过滤（EE）

如果两条边共享一个或两个端点：

```cpp
// EE 候选：两条边不能共享任何端点
if (ea0 == eb0 || ea0 == eb1 || ea1 == eb0 || ea1 == eb1) {
    skip;
}
```

#### 3. 同体 obstacle 过滤（可选）

如果两个原语都属于同一个 obstacle body（静态的），不需要检测。

#### 4. self-contact 跳过（Day 2）

Day 2 不做 self-contact。如果两个原语属于同一个 tet body，跳过。

```cpp
if (vertex_body[point_idx] == triangle_body[tri_idx]
    && vertex_body[point_idx] != kObstacleBodyId) {
    skip;  // same deformable body -> self-contact, skip for Day 2
}
```

### 候选对数据结构

```cpp
struct PTCandidate {
    std::size_t point_surface_idx;    // collision mesh 中的表面顶点索引
    std::size_t triangle_idx;         // collision mesh 中的三角形索引
};

struct EECandidate {
    std::size_t edge_a_idx;           // collision mesh 中的边索引
    std::size_t edge_b_idx;
};

struct CollisionCandidates {
    std::vector<PTCandidate> pt_candidates{};
    std::vector<EECandidate> ee_candidates{};
};
```

### AABB 加速（可选优化）

如果 brute force 太慢，可以加最简单的 AABB 预过滤：

1. 对每个三角形/边计算 AABB（带 padding = `dhat`）
2. PT 候选：只在点落在三角形 AABB 内时保留
3. EE 候选：只在两条边的 AABB 重叠时保留

Day 2 建议先不做 AABB，纯 brute force 验证正确性。如果 demo 场景太慢再加。

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | CollisionMesh 结构 + build 函数 |
| `src/rtr/system/physics/ipc/contact/collision_candidates.hpp` | 候选对结构 + brute force 生成 + 过滤 |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/contact/collision_mesh_test.cpp` | 验证 vertex_map 正确、边提取去重、body 标记 |
| `test/system/physics/ipc/contact/collision_candidates_test.cpp` | 验证共享顶点/边过滤、同体过滤 |
