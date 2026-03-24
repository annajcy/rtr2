# Phase 2: Collision Mesh + Broad Phase

> Naming note after the converter refactor:
> `TetSurfaceResult` -> `TetSurfaceMapping`
> `extract_tet_surface()` -> `build_tet_surface_mapping()`

## 目标

把当前仓库里“可求解的全局 DOF 向量”和“接触检测需要的表面 primitive”桥接起来，并给 Phase 3 barrier / Phase 5 CCD 提供**统一、稳定、可测试**的候选对输入。

这一阶段必须解决两件事：

1. 建立统一 collision mesh，把 tet surface 和 static obstacle surface 拼成一棵统一的接触表面
2. 在这个 collision mesh 上生成第一版最小可用 broad phase 候选集

Day 2 这里的重点是**接口和正确性**，不是性能。

---

## 为什么这一层必须单独存在

当前 `IPCState.x` 只存 tet body 的可求解顶点位置，接触层真正需要的却是：

- 表面顶点
- 表面边
- 表面三角形
- 每个 primitive 的 body 归属
- 每个顶点当前坐标应该从哪里读取

这些信息既不属于：

- `geometry/`：因为 geometry 只接受局部坐标，不关心拓扑和 body ownership
- `IPCState`：因为 `IPCState` 只关心可求解的全局 DOF
- `TetSurfaceMapping`：因为它只描述单个 tet body 的表面，不负责跨 body 合并

所以 Day 2 需要单独的 `contact/` 桥接层。

---

## 当前仓库给这一步带来的约束

### 约束 1：Obstacle 不能进 `IPCState`

Day 2 已在总计划中明确：

- tet 顶点位置来自 `IPCState.x`
- obstacle 顶点是静态世界空间位置
- obstacle 不参与 global DOF

所以 collision mesh **不能**只有一份 `vertex_map -> global_vertex_id`。它必须能同时表达：

- deformable surface vertex：坐标来自 `state.x`
- obstacle surface vertex：坐标来自 obstacle 自己存的静态位置

### 约束 2：必须复用 `TetSurfaceMapping`

当前仓库已经有：

- `TetSurfaceMapping`
- `build_tet_surface_mapping()`

Phase 2 不应该再重新发明 tet surface 提取逻辑。collision mesh 的职责是：

- 读取每个 tet body 的 `TetSurfaceMapping`
- 合并进统一表面
- 给这些表面 primitive 补齐 ownership、边信息和坐标读取方式

### 约束 3：Day 2 只做 tet vs static obstacle

Day 2 的 broad phase 不需要支持：

- self-contact
- tet vs tet 多体接触
- obstacle vs obstacle

所以候选生成必须围绕**deformable side vs obstacle side**来组织，而不是盲目枚举全 mesh 的所有 pair。

---

## Collision Mesh 设计

### 核心设计原则

collision mesh 不直接保存“当前所有顶点坐标数组”。它保存：

1. 统一的表面 primitive 拓扑
2. 每个 primitive 的 body 归属
3. 每个顶点的坐标来源

也就是说，collision mesh 是一个**统一表面索引空间 + 位置读取桥接层**。

### 推荐数据结构

```cpp
enum class CollisionVertexKind {
    Deformable,
    Obstacle,
};

struct CollisionVertex {
    IPCBodyID body_id{0};
    CollisionVertexKind kind{CollisionVertexKind::Deformable};

    // 顶点在所属 body 内的局部索引
    std::size_t body_vertex_index{0};

    // 仅对 deformable 顶点有效：在 IPCState 中的 global vertex index
    std::optional<std::size_t> global_vertex_index{};

    // 仅对 obstacle 顶点有效：静态世界空间位置
    Eigen::Vector3d static_position{Eigen::Vector3d::Zero()};
};

struct CollisionTriangle {
    std::array<std::size_t, 3> vertices{};
    IPCBodyID body_id{0};
};

struct CollisionEdge {
    std::array<std::size_t, 2> vertices{};
    IPCBodyID body_id{0};
};

struct CollisionMesh {
    std::vector<CollisionVertex> vertices{};
    std::vector<CollisionTriangle> triangles{};
    std::vector<CollisionEdge> edges{};

    // 预分组好的 primitive index，保证 broad phase 不用每次重新扫 body type
    std::vector<std::size_t> deformable_vertex_indices{};
    std::vector<std::size_t> obstacle_vertex_indices{};
    std::vector<std::size_t> deformable_triangle_indices{};
    std::vector<std::size_t> obstacle_triangle_indices{};
    std::vector<std::size_t> deformable_edge_indices{};
    std::vector<std::size_t> obstacle_edge_indices{};

    std::size_t vertex_count() const { return vertices.size(); }
    std::size_t triangle_count() const { return triangles.size(); }
    std::size_t edge_count() const { return edges.size(); }
};
```

这个版本比最初的 `vertex_map + triangle_body + edge_body` 更适合当前仓库，因为 obstacle 顶点根本没有全局 DOF。

### 顶点位置读取协议

这一步要把“顶点坐标来自哪里”定死，避免后面 barrier / CCD 再各写一遍。

推荐 contact 层补一个统一 helper：

```cpp
Eigen::Vector3d read_collision_vertex_position(
    const CollisionMesh& mesh,
    const IPCState& state,
    std::size_t surface_vertex_idx
);
```

规则：

- `kind == Deformable`：从 `state.x.segment<3>(3 * global_vertex_index)` 读取
- `kind == Obstacle`：直接返回 `static_position`

这样后面的 broad phase、barrier wrapper、CCD wrapper 都走同一个坐标入口。

### 为什么这里就要带 `global_vertex_index`

Phase 3 barrier 装配时，local primitive 的参与顶点需要映射回 global DOF。

如果 collision mesh 现在不保留 deformable 顶点到 global vertex index 的映射，那么后面 barrier / CCD 就必须回头再查 body + offset，接口会变得更脆弱。

所以推荐直接把这份映射放到 `CollisionVertex` 里。

---

## Collision Mesh 构建流程

### 推荐入口

```cpp
CollisionMesh build_collision_mesh(const IPCSystem& system);
```

Phase 2 可以接受 contact 层先直接读 `IPCSystem`，因为当前 body 容器都在系统内部。重点是：

- collision mesh 自己是纯数据结果
- 后面的 broad phase / barrier / CCD 只依赖 `CollisionMesh + IPCState`

### 构建步骤

#### 1. 追加所有 tet body 的表面 primitive

对每个启用的 tet body：

1. 调用 `build_tet_surface_mapping(body)`
2. 为 `surface_vertex_ids` 中的每个局部顶点创建一个 `CollisionVertex`
3. 计算其 global vertex index：

```cpp
global_vertex_index = body.info.dof_offset / 3 + local_vertex_id;
```

4. 把 `surface_indices` 按三元组转成 `CollisionTriangle`
5. 三角形顶点索引写入 collision mesh 自己的 surface-local 顶点编号
6. 记录这些 primitive 的 `body_id`

#### 2. 追加所有 obstacle body 的表面 primitive

对每个启用的 obstacle body：

1. 直接读取 obstacle 的 `positions` 和 `triangles`
2. 为每个 obstacle 顶点创建 `CollisionVertex`
3. 设置：
   - `kind = Obstacle`
   - `body_vertex_index = obstacle local vertex id`
   - `global_vertex_index = nullopt`
   - `static_position = obstacle.positions[i]`
4. 把 obstacle 三角形追加为 `CollisionTriangle`

#### 3. 从三角形提取边并去重

对每个三角形 `(v0, v1, v2)`：

- 加入 `(min(v0, v1), max(v0, v1))`
- 加入 `(min(v1, v2), max(v1, v2))`
- 加入 `(min(v0, v2), max(v0, v2))`

推荐实现：

- 先用 `std::vector<std::tuple<body_id, a, b>>`
- 最后 `sort + unique`

把 `body_id` 一起放进 key 很重要。因为 Day 2 只做跨 body 接触，但边的归属仍然要显式保留给 broad phase 过滤和后续调试。

#### 4. 预填 primitive 分类索引

构建完后立刻填：

- `deformable_vertex_indices`
- `obstacle_vertex_indices`
- `deformable_triangle_indices`
- `obstacle_triangle_indices`
- `deformable_edge_indices`
- `obstacle_edge_indices`

这样 broad phase 的 loops 可以直接按“物体类别”组织，避免每次重新读 body type。

---

## ObstacleBody 在这一阶段必须补到什么程度

当前 `ObstacleBody` 还是占位类型。为了让 collision mesh 落地，Phase 2 至少要把它升级为：

```cpp
struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};
    std::vector<Eigen::Vector3d> positions{};
    std::vector<std::array<std::size_t, 3>> triangles{};
};
```

Day 2 这一阶段先不要求：

- obstacle 自己缓存边
- kinematic obstacle
- 法线或 UV

因为 collision mesh 可以从三角面自己提边。

---

## Broad Phase 设计

### Day 2 原则：先按“正确 + 可验证”设计

Day 2 broad phase 默认走 brute force，复杂度不是重点。真正重要的是：

- 候选原语组合是完整的
- 过滤规则是明确的
- 输出顺序是稳定的

### Day 2 真正需要枚举的 primitive 组合

由于 Day 2 只做 tet vs static obstacle，所以 broad phase 不应该枚举“全表面任意 pair”，而应当只生成三类有效组合：

1. deformable point vs obstacle triangle
2. obstacle point vs deformable triangle
3. deformable edge vs obstacle edge

也就是说：

- PT 必须双向
- EE 只做 deformable edge vs obstacle edge
- 不做 deformable vs deformable
- 不做 obstacle vs obstacle

### 为什么 PT 要双向

如果只做“deformable point vs obstacle triangle”，会漏掉“obstacle vertex vs deformable triangle”这类最近特征关系。

Day 2 虽然只做 static obstacle，但 broad phase 仍应把三角网格间的局部 primitive 配对写完整：

- PT 双向覆盖 vertex-face 型接近
- EE 覆盖 edge-edge 型接近

这样 Phase 3 barrier 不需要再临时补“方向性例外”。

---

## Broad Phase 数据结构

```cpp
struct PTCandidate {
    std::size_t point_vertex_idx{0};   // CollisionMesh::vertices 索引
    std::size_t triangle_idx{0};       // CollisionMesh::triangles 索引
};

struct EECandidate {
    std::size_t edge_a_idx{0};         // CollisionMesh::edges 索引
    std::size_t edge_b_idx{0};
};

struct CollisionCandidates {
    std::vector<PTCandidate> pt_candidates{};
    std::vector<EECandidate> ee_candidates{};
};
```

这里候选只记录 collision mesh 中的 primitive index，不额外重复 body 信息。body ownership 从 `CollisionMesh` 上查即可。

---

## 候选生成接口

### 推荐配置

```cpp
struct BroadPhaseConfig {
    bool enable_aabb_prefilter{false};
    double aabb_padding{0.0}; // 推荐后续接 barrier 时传 sqrt(dhat_squared)
};
```

### 推荐入口

```cpp
CollisionCandidates build_collision_candidates(
    const CollisionMesh& mesh,
    const IPCState& state,
    const BroadPhaseConfig& config = {}
);
```

即使第一版不做 AABB 预过滤，也建议把 `state` 和 `config` 先放进接口里，这样之后加最简单的 AABB broad phase 不需要改调用面。

---

## Broad Phase 枚举顺序

为了保证测试稳定、日志稳定、后续 debug 容易复现，候选生成顺序应固定。

### PT

先生成：

1. `deformable_vertex_indices` × `obstacle_triangle_indices`
2. `obstacle_vertex_indices` × `deformable_triangle_indices`

都按索引升序双重循环。

### EE

生成：

1. `deformable_edge_indices` × `obstacle_edge_indices`

同样按升序双重循环。

这样能保证同一输入 mesh 每次都得到相同顺序的候选集。

---

## 候选过滤规则

虽然 Day 2 只做 tet vs obstacle，但过滤规则仍应明确写死，避免 Phase 3 / Phase 5 再重复一遍。

### PT 过滤

#### 1. 共享顶点过滤

如果 point 正好就是 triangle 某个顶点，跳过：

```cpp
if (point_idx == tri[0] || point_idx == tri[1] || point_idx == tri[2]) {
    skip;
}
```

对当前 tet-vs-obstacle 路径，这种情况通常不会出现，但保留这条规则能让接口更稳。

#### 2. 同 body 过滤

如果 point 和 triangle 属于同一个 body，跳过：

```cpp
if (mesh.vertices[point_idx].body_id == mesh.triangles[tri_idx].body_id) {
    skip;
}
```

这条规则直接把 Day 2 的 self-contact 排除掉。

#### 3. obstacle-obstacle 过滤

如果 point 和 triangle 都来自 obstacle，跳过。

在当前枚举策略下这类 pair 本来不会生成，但 broad phase helper 内部仍建议保留显式断言或过滤，避免未来改 loop 时漏掉。

### EE 过滤

#### 1. 共享端点过滤

如果两条边共享任意端点，跳过：

```cpp
if (ea0 == eb0 || ea0 == eb1 || ea1 == eb0 || ea1 == eb1) {
    skip;
}
```

#### 2. 同 body 过滤

如果两条边属于同一个 body，跳过。

Day 2 不做 self-contact，这条必须有。

#### 3. obstacle-obstacle 过滤

两条边都来自 obstacle，跳过。

在当前枚举设计下也不会生成，但建议保留保护。

---

## AABB 预过滤：作为可选优化预留

Day 2 默认不开启 AABB broad phase，但接口应为后续预留。

### 推荐的最小 AABB 策略

如果 `enable_aabb_prefilter == true`：

1. 对 triangle / edge 在当前坐标下计算 AABB
2. AABB 各轴扩张 `aabb_padding`
3. PT：只有 point 落在 triangle AABB 内才保留
4. EE：只有两条边的 AABB 重叠才保留

这一步只做**保守过滤**，不做更复杂的 BVH / spatial hash。

Day 2 默认值：

- `enable_aabb_prefilter = false`
- 先用纯 brute force 跑通

---

## 与后续 Phase 的接口约定

### 给 Phase 3 barrier

Phase 3 需要：

- 从 `PTCandidate` / `EECandidate` 构造局部 `Distance::Input`
- 从 `CollisionVertex` 读当前坐标
- 对 deformable 顶点建立 local-to-global DOF map

所以 Phase 2 必须保证：

- candidate 只用 collision mesh primitive index 表达
- collision mesh 已经包含足够的 vertex source / ownership 信息

### 给 Phase 5 CCD

Phase 5 需要：

- 在 `x + alpha * dx` 下重复读取局部 primitive 坐标
- 对同一批 candidate 做安全步长判断

因此 collision mesh 和 candidates 都必须是**与当前 `alpha` 无关的静态拓扑结果**。

也就是说：

- collision mesh 不存 deformable 顶点的“当前坐标快照”
- deformable 顶点位置始终按需从 `IPCState` 读
- obstacle 顶点位置则直接存静态世界坐标

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | `CollisionVertex / CollisionTriangle / CollisionEdge / CollisionMesh` + build / read-position helper |
| `src/rtr/system/physics/ipc/contact/collision_candidates.hpp` | `BroadPhaseConfig`、`PTCandidate`、`EECandidate`、`CollisionCandidates`、brute-force 生成与过滤 |

如果实现中发现 AABB helper 会重复，可允许加一个很薄的：

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/contact/contact_aabb.hpp` | point / edge / triangle AABB 计算与 overlap helper |

但 Day 2 第一版不应先拆太多 util。

---

## 测试策略

### `collision_mesh_test.cpp`

至少验证：

1. tet body 的 `TetSurfaceMapping` 能正确转成 deformable `CollisionVertex`
2. deformable 顶点的 `global_vertex_index` 正确
3. obstacle 顶点的 `static_position` 被正确保留
4. triangle append 后，edge 提取能稳定去重
5. `deformable_*` / `obstacle_*` primitive index 分组正确
6. `read_collision_vertex_position(...)` 对 deformable / obstacle 都返回正确坐标

### `collision_candidates_test.cpp`

至少验证：

1. 只生成 Day 2 需要的三类跨体组合
2. PT 双向都能生成
3. 同 body PT 不会进入候选
4. 共享端点的 EE 被过滤
5. obstacle-obstacle pair 不会进入候选
6. 候选顺序稳定，可用固定输入做精确比对

### 如果实现了 AABB 预过滤

额外验证：

1. AABB overlap 为假时 pair 被安全剔除
2. overlap 为真时 pair 至少不会被误删

---

## Phase 2 的完成标准

做到下面这些，这一阶段才算完成：

- `ObstacleBody` 至少支持静态三角网格顶点 + 面片
- `CollisionMesh` 能同时承载 deformable 和 obstacle surface
- deformable 顶点能从 `IPCState.x` 正确读位置
- obstacle 顶点能从静态位置正确读位置
- 边提取稳定去重
- broad phase 只生成 Day 2 所需的 PT 双向 + EE 跨体候选
- 共享端点 / 同 body / obstacle-obstacle 过滤正确
- 测试覆盖 collision mesh 构建和 broad phase 过滤

如果这一步没有定稳，后面的 barrier 和 CCD 就会不断反复修改 primitive 映射逻辑，代价会比现在补细节更高。
