# Day 2 Overview

## 目标

基于 Day 1 已经跑通的无接触 tet FEM 隐式求解链路，在当前仓库里补上第一版 **obstacle contact**：

**`TetBody + static ObstacleBody -> collision geometry -> barrier energy -> collision-safe Newton step -> stable non-penetration`**

Day 2 的完成标准不是“把若干 contact 文件建出来”，而是让现有 `IPCSystem` 真正能够在一个完整时间步里处理：

1. tet body 自由下落
2. 与静态障碍物接近时 barrier 被激活
3. Newton 方向经过 CCD/安全步长约束
4. line search 不会把状态推进到穿透
5. 最终 demo 中 block 落到地面后停住或稳定贴近，不出现明显穿透/NaN

---

## 当前仓库的真实起点

这份 Day 2 计划必须以仓库当前状态为前提，而不是以理想化 IPC 架构为前提。现在已经存在且可以直接复用的东西如下。

### 已完成的 Day 1 骨架

| 模块 | 文件 | 现状 |
|------|------|------|
| 全局状态 | `src/rtr/system/physics/ipc/core/ipc_state.hpp` | `x / x_prev / v / mass_diag` 已完整 |
| Tet body | `src/rtr/system/physics/ipc/model/tet_body.hpp` | rest shape、`Dm_inv`、体积、mass lumping 已完成 |
| Tet 表面映射 | `src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp` | `TetSurfaceMapping` + `build_tet_surface_mapping()` 已可直接复用 |
| 能量项 | `energy/inertial_energy.hpp` `gravity_energy.hpp` `material_energy.hpp` | 已进入总能量装配 |
| Newton solver | `solver/newton_solver.hpp` | reduced Hessian、正则化、回溯 line search 已完成 |
| Line search | `solver/line_search.hpp` | 只有 Armijo，没有 `alpha_max` / CCD 上界 |
| IPCSystem | `core/ipc_system.hpp` | 多 tet body、rebuild、scene dirty 标记已完成 |
| Scene bridge | `src/rtr/framework/integration/physics/ipc_scene_sync.hpp` | tet component <-> IPCSystem 同步已完成 |
| 编辑器 demo | `examples/editor/ipc_fixed_end_block_editor.cpp` | 已有固定端 block demo |
| 测试 | `test/system/physics/ipc/*.cpp` | 当前已有 13 个 IPC 测试文件 |

### 当前明确还没有的东西

| 缺口 | 当前文件 | 现状 |
|------|----------|------|
| Obstacle 几何 | `model/obstacle_body.hpp` | 只有 `IPCBodyInfo` 占位，没有顶点/三角形 |
| 接触几何模块 | `ipc/geometry/` | 目录不存在 |
| 碰撞候选/mesh | `ipc/contact/` | 目录不存在 |
| CCD 模块 | `ipc/ccd/` | 目录不存在 |
| Contact 装配 | `core/ipc_system.hpp` | 总能量只有惯性/重力/弹性 |
| Solver 钩子 | `newton_solver.hpp` | `NewtonProblem` 没有安全步长回调 |
| Line search 上界 | `line_search.hpp` | 初始步长固定从 `alpha_init` 开始，无碰撞约束 |

这意味着 Day 2 不需要“重构一套新的 solver”，而是要沿着现有接口把 contact 以最小侵入的方式插进去。

---

## Day 2 的范围界定

### 要做

只做 **tet body 对 static obstacle 的第一版无摩擦接触**。

具体来说：

- deformable side：现有 `TetBody`
- obstacle side：新的静态三角网格 `ObstacleBody`
- 接触原语：PT 和 EE
- 接触势能：barrier
- 线搜索安全性：conservative CCD / 安全步长
- broad phase：先 brute force 或非常简单的 AABB 过滤

### 不做

- self-contact
- deformable-deformable 接触优先优化
- friction
- kinematic obstacle
- obstacle 进入全局 DOF
- BVH / spatial hash / 高性能 broad phase
- adaptive barrier stiffness
- cloth / shell

这份边界非常重要，因为当前 `IPCState` 只管理 tet 顶点 DOF。只做 static obstacle，才能保证 Day 2 基本不需要动 `IPCState` 的核心设计。

---

## 基于现有代码的设计原则

### 1. 不改 `IPCState` 的 DOF 语义

`src/rtr/system/physics/ipc/core/ipc_state.hpp` 现在默认“所有 DOF 都是可求解顶点位置”。Day 2 不要把 obstacle 顶点塞进 `state.x`。

结论：

- obstacle 顶点只存于 `ObstacleBody.positions`
- barrier gradient/Hessian 只装配到 deformable 顶点对应的全局 DOF
- collision mesh 必须能同时读取：
  - deformable 顶点：来自 `state.x`
  - obstacle 顶点：来自 obstacle 自己的世界空间位置

### 2. 尽量复用现有 `TetSurfaceMapping`

`build_tet_surface_mapping()` 已经能给出单个 tet body 的表面三角形与表面顶点集合。Day 2 不要重新发明 tet 表面提取。

collision mesh 的工作应是：

- 读取每个 tet body 的 `TetSurfaceMapping`
- 拼成全局接触表面
- 补边信息、body ownership、顶点读取方式

### 3. Contact 模块尽量做成无状态纯函数

当前 IPC 的风格明显偏 header-only / stateless helper：

- `InertialEnergy::compute_*`
- `GravityEnergy::compute_*`
- `material_energy_variant::compute_*`
- `backtracking_line_search(...)`

Day 2 的 geometry / barrier / CCD 最好也延续这套风格。状态只留在 `IPCSystem` 里，算法模块本身尽量纯函数化，测试也更直接。

### 4. Solver 只开一个必要接口

当前 `solve(...)` 设计已经够用。Day 2 不要把 contact 逻辑塞进 solver 内部。

建议只新增一个最小钩子：

```cpp
std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> compute_max_stepsize{};
```

然后 solver 在 line search 前查询 `alpha_max`。这样 contact 仍然由 `IPCSystem` 控制，solver 只负责“遵守上界”。

---

## Day 2 的代码落点

### 新增目录

```text
src/rtr/system/physics/ipc/
├── geometry/
├── contact/
└── ccd/
```

### 新增核心文件

| 模块 | 计划文件 | 作用 |
|------|----------|------|
| 几何距离 | `geometry/point_point_distance.hpp` | PP 距离平方、梯度、Hessian |
| 几何距离 | `geometry/point_edge_distance.hpp` | PE 距离平方、梯度、Hessian |
| 几何距离 | `geometry/point_triangle_distance.hpp` | PT 距离平方、区域分类、梯度、Hessian |
| 几何距离 | `geometry/edge_edge_distance.hpp` | EE 距离平方、退化处理、梯度、Hessian |
| 统一接触表面 | `contact/collision_mesh.hpp` | 拼接 tet surface + obstacle surface |
| 候选对 | `contact/collision_candidates.hpp` | PT/EE brute-force 候选生成和过滤 |
| 标量 barrier | `contact/barrier_function.hpp` | `b(s, s_hat)` 及导数 |
| barrier 装配 | `contact/barrier_potential.hpp` | 局部 12-DOF -> 全局装配 |
| CCD | `ccd/collision_free_stepsize.hpp` | 对当前 Newton 方向给出安全步长 |

### 修改已有文件

| 文件 | 必改点 |
|------|--------|
| `src/rtr/system/physics/ipc/model/obstacle_body.hpp` | 从占位升级为静态三角网格 obstacle |
| `src/rtr/system/physics/ipc/core/ipc_system.hpp` | obstacle 容器、collision mesh、barrier 装配、CCD 集成 |
| `src/rtr/system/physics/ipc/solver/newton_solver.hpp` | `compute_max_stepsize` 回调 |
| `src/rtr/system/physics/ipc/solver/line_search.hpp` | `alpha_max` 参数 |
| `examples/CMakeLists.txt` | 新增 contact demo |
| `test/CMakeLists.txt` | 新增 geometry/contact/ccd 测试 |

---

## 推荐实现顺序

这里的顺序不是理论上的依赖顺序，而是考虑了当前仓库“编译随时可过、每一步都可测”的最小风险路径。

### Phase 1: 几何距离模块

先做纯数学层：

- `PP`
- `PE`
- `PT`
- `EE`

要求：

- 每个函数都先在局部坐标上闭环
- 单独通过有限差分测试
- 先不接触 `IPCSystem`

这样可以把最容易出数值错的部分单独关掉。

### Phase 2: Collision mesh + 候选集

接上现有 `TetSurfaceMapping`：

- 把 tet 表面三角形汇总
- 为 obstacle 补统一表示
- 从三角面提取边
- 生成 PT/EE 候选

要求：

- 不涉及 energy/solver
- 只验证拓扑和过滤逻辑

### Phase 3: Barrier potential

把几何距离结果变成能量/梯度/Hessian：

- 标量 barrier 先单测
- 再做单对 PT/EE 的局部 barrier
- 最后做全局装配

注意这一步仍然不依赖 solver 结构修改，先保证 barrier 本身数学正确。

### Phase 4: ObstacleBody

把 `ObstacleBody` 从空壳补成真实静态网格体：

- 顶点
- 三角形
- 边
- `generate_ground_plane()`

这里与 collision mesh 一起闭环，保证“能构建接触表面但还没进入求解器”。

### Phase 5: CCD / 安全步长

在当前仓库里，Day 2 不要求解析 CCD 一步到位。更现实的目标是：

- 能给出 conservative 的 `alpha_max`
- 让 line search 永远不跨过碰撞

只要能稳定约束 demo，就已经足够支撑 Day 2。

### Phase 6: 接入 `IPCSystem`

这是第一次真正修改主循环：

1. 进入 step 前或 step 初始阶段构建 collision mesh
2. 在 energy/gradient/Hessian 里加 barrier
3. 在 Newton 方向上计算 `alpha_max`
4. line search 遵守 `alpha_max`

这一步应尽量最小化修改范围，不要重写 solver 流程。

### Phase 7: Demo + 系统测试

最后才做完整场景验证：

- 地面接触 editor demo
- smoke test
- 长时间步进稳定性检查

---

## 与当前 `IPCSystem` 的精确对接点

当前 `IPCSystem.step()` 的主要流程是：

```text
rebuild_runtime_state()
-> x_prev = x
-> compute_x_hat()
-> solve(...)
-> v = (x - x_prev) / dt
```

Day 2 之后应变成：

```text
rebuild_runtime_state()                    // 仅 tet DOF rebuild
-> x_prev = x
-> compute_x_hat()
-> build collision mesh / refresh contact view
-> solve(
       energy = inertial + gravity + elastic + barrier,
       gradient = ...
       hessian = ...
       max_step = contact-safe alpha
   )
-> v = (x - x_prev) / dt
```

这里最关键的三个插入点是：

### 插入点 A: `IPCConfig`

新增 contact 参数：

```cpp
double dhat_squared{0.01};
double barrier_stiffness{1e4};
bool enable_contact{true};
```

### 插入点 B: `IPCSystem` 成员

新增：

```cpp
std::unordered_map<IPCBodyID, ObstacleBody> m_obstacle_bodies{};
CollisionMesh m_collision_mesh{};
CollisionCandidates m_candidates{};
```

### 插入点 C: `NewtonProblem`

当前只有：

- `compute_energy`
- `compute_gradient`
- `compute_hessian_triplets`

Day 2 新增：

```cpp
compute_max_stepsize
```

让 `IPCSystem` 负责计算 contact-safe step，solver 只是消费它。

---

## 当前仓库下的最小 demo 目标

现有 demo 是 [`examples/editor/ipc_fixed_end_block_editor.cpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/examples/editor/ipc_fixed_end_block_editor.cpp)，它已经提供了：

- editor runtime
- camera / light / ground visual
- `IPCTetComponent`
- deformable mesh 可视化

Day 2 最划算的做法不是完全重写，而是复制这个 demo，做一个新的：

- `ipc_ground_contact_editor.cpp`

与 Day 1 demo 的唯一区别：

- 不再固定一端
- ground 从“仅渲染物体”升级为真正的 `ObstacleBody`
- 让 tet block 在 gravity 下自由落下并接触地面

如果这个 demo 稳定，Day 2 就算真正闭环。

---

## 风险点与对应策略

### 风险 1: 距离函数数学实现复杂，容易把错误带进 barrier

策略：

- Phase 1 单独完成
- 全部做有限差分
- 先不和 `IPCSystem` 联调

### 风险 2: barrier 做出来但 line search 仍能穿透

策略：

- 把 CCD 视为 Day 2 必做，不拆到明天
- 最低要求是 conservative `alpha_max`

### 风险 3: obstacle 不在 DOF 内导致装配代码分叉过多

策略：

- 在 `CollisionMesh` 提供统一的“取顶点位置”和“是否可装配到全局 DOF”的接口
- 不让 barrier 层直接感知 `TetBody` / `ObstacleBody`

### 风险 4: 过早追求高性能 broad phase

策略：

- Day 2 只要求正确
- 小场景先 brute force
- 以后再做 BVH / spatial hash

---

## Day 2 完成时必须成立的结果

### 功能结果

- tet block 落到地面后不会明显穿透
- 接近障碍物时 barrier energy 非零且增大
- line search 的步长确实受 contact-safe 上界约束
- 多步运行不出现 NaN / inf

### 代码结果

- `ObstacleBody` 不再是占位类型
- `IPCSystem` 可以注册 obstacle
- contact 路径不破坏现有无接触路径
- `enable_contact=false` 时，Day 1 行为应保持不变

### 测试结果

- 几何距离 FD 测试通过
- barrier 标量与局部装配测试通过
- 至少一个 contact smoke test 通过

---

## Day 3 的自然衔接点

如果 Day 2 按上述结构完成，Day 3 加 self-contact 时就有明确插入点：

- geometry 层：无需大改
- collision mesh 层：新增 adjacency / same-body topology filtering
- candidate 层：放开同一 tet body 的非邻接候选
- barrier / CCD / solver：基本复用

也就是说，Day 2 的价值不是只做一个地面 demo，而是把接触框架以一种能继续长大的方式插进当前仓库。
