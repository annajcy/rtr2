# Day 3 Overview

## 目标

完成 IPC 接触管线的端到端闭环：

**`TetBody + ObstacleBody -> collision mesh -> barrier energy -> CCD-safe Newton step -> stable non-penetration demo -> system integration -> 最小完整交付`**

Day 3 是 IPC 5 天计划的最后一个实现日。Day 3 结束时必须交付一个可运行的"研究型核心 IPC 版本"。

---

## 当前仓库的真实起点

### Day 1 已完成（已提交）

| 模块 | 现状 |
|------|------|
| `IPCState` | 统一 DOF、mass_diag 完整 |
| `TetBody` | rest shape、Dm_inv、体积、mass lumping |
| `TetSurfaceMapping` | 表面提取可复用 |
| 能量项 | inertial + gravity + elastic 已装配 |
| Newton solver | reduced Hessian、正则化、回溯 line search |
| `IPCSystem` | 多 tet body、rebuild、scene dirty |
| Scene bridge | tet component <-> IPCSystem 同步 |
| Editor demo | 固定端 block 下垂 demo |
| 测试 | 13 个通过的测试 |

### Day 2 已完成（已提交）

| 模块 | 现状 |
|------|------|
| 几何距离 | PP/PE/PT/EE 距离平方 + gradient + Hessian |
| Distance concept | 统一的 `Distance` concept + `DistanceResult` |
| 几何距离测试 | FD 检查全部通过 |

### Day 2 进行中（未提交）

| 模块 | 现状 |
|------|------|
| `CollisionMesh` | 文件已创建，功能待完善 |
| `CollisionCandidates` | 文件已创建，功能待完善 |
| `ObstacleBody` | 已有修改但未完成 |
| `IPCSystem` | 已有修改但未集成 contact |
| `test/CMakeLists.txt` | 已添加新测试目标 |

### 明确还没有的东西

| 缺口 | 现状 |
|------|------|
| Barrier function | 不存在 |
| Barrier potential（全局装配） | 不存在 |
| CCD / 安全步长 | 不存在 |
| IPCSystem contact 装配 | 总能量只有惯性/重力/弹性 |
| NewtonProblem `compute_max_stepsize` | 不存在 |
| Line search `alpha_max` | 不存在 |
| Contact demo | 不存在 |
| PhysicsSystem 接入 | IPCSystem 未接入主运行时 |

---

## Day 3 的范围界定

### 必做（核心 contact pipeline）

1. **完成 collision mesh + candidates**（Day 2 Phase 2 收尾）
2. **Barrier function + potential**（标量 barrier + 链式法则 + PSD projection + 全局装配）
3. **ObstacleBody 升级**（静态三角网格 + ground plane generator）
4. **CCD / conservative 安全步长**（backtracking CCD + line search 集成）
5. **IPCSystem contact 集成**（barrier 进入总能量 + CCD 约束 line search）
6. **Contact smoke test**（tet block 落到地面不穿透）

### 必做（系统收尾）

7. **PhysicsSystem 接入**（IPCSystem 可选接入主运行时循环）
8. **Contact demo**（ground contact editor demo 可运行）
9. **参数面板**（最小配置：dt, kappa, dhat, Newton iters, material params）
10. **核心 smoke tests 补齐**
11. **已知限制说明 + 下一步待办**

### 不做

- self-contact
- deformable-deformable 接触
- friction
- kinematic obstacle（运动障碍物）
- BVH / spatial hash 高性能 broad phase
- adaptive barrier stiffness
- cloth / ShellBody
- 完整 hyperelastic 材料库
- 生产级 profiling / 并行优化

---

## 执行顺序

Day 3 分为两个大阶段：上午完成 contact pipeline，下午完成系统集成和交付。

```text
Phase 1: 完成 Collision Mesh + Candidates           (~1 hr)
Phase 2: Barrier Function + Potential                (~2 hr)
Phase 3: ObstacleBody 升级                           (~1 hr)
Phase 4: CCD + 安全步长                              (~1.5 hr)
Phase 5: IPCSystem Contact 集成                      (~1.5 hr)
Phase 6: PhysicsSystem 接入 + Demo                   (~1.5 hr)
Phase 7: 测试、调参、收尾                             (~1.5 hr)
```

### 关键原则

1. **每个 Phase 独立可测**：barrier 不依赖 CCD 来测试，CCD 不依赖 IPCSystem 来测试
2. **先测后接**：每个模块先通过单元测试再接入 IPCSystem
3. **最小侵入**：不重写现有 solver，只在必要处开接口
4. **正确优先**：Day 3 不追求性能，brute force + backtracking CCD 足够

---

## Day 3 的代码落点

### 新增文件

| 文件 | 说明 |
|------|------|
| `src/rtr/system/physics/ipc/contact/barrier_function.hpp` | 标量 barrier $b(s, \hat{s})$ 及一二阶导数 |
| `src/rtr/system/physics/ipc/contact/barrier_potential.hpp` | 基于 Distance concept 的 pair barrier + 全局装配 |
| `src/rtr/system/physics/ipc/ccd/ccd_config.hpp` | CCD 参数 |
| `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp` | conservative 安全步长 |
| `examples/editor/ipc_ground_contact_editor.cpp` | contact demo |
| `test/system/physics/ipc/contact/barrier_function_test.cpp` | 标量 barrier FD 检查 |
| `test/system/physics/ipc/contact/barrier_potential_test.cpp` | pair barrier gradient FD 检查 |
| `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp` | CCD 安全步长测试 |
| `test/system/physics/ipc/ipc_contact_smoke_test.cpp` | 系统级 contact smoke test |

### 修改文件

| 文件 | 修改内容 |
|------|---------|
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | 完善 obstacle 顶点支持 |
| `src/rtr/system/physics/ipc/contact/collision_candidates.hpp` | 完善候选过滤 |
| `src/rtr/system/physics/ipc/model/obstacle_body.hpp` | 从占位升级为完整实现 |
| `src/rtr/system/physics/ipc/core/ipc_system.hpp` | obstacle 管理、barrier 装配、CCD 集成 |
| `src/rtr/system/physics/ipc/solver/newton_solver.hpp` | `NewtonProblem` 新增 `compute_max_stepsize` |
| `src/rtr/system/physics/ipc/solver/line_search.hpp` | 新增 `alpha_max` 参数 |
| `examples/CMakeLists.txt` | 新增 demo target |
| `test/CMakeLists.txt` | 新增测试目标 |

---

## 与 5 天计划的对照

| 原始 5 天计划 | Day 3 覆盖情况 |
|---------------|---------------|
| Day 3: 接触几何、Barrier、Obstacle Contact | Phase 2-3-5 覆盖 |
| Day 4: CCD + collision-free line search | Phase 4-5 覆盖 |
| Day 5: 系统收尾、Demo、最小完整交付 | Phase 6-7 覆盖 |

Day 3 实质上合并了原始计划的 Day 3 + Day 4 + Day 5 的核心交付内容。这是因为：

1. Day 2 已经完成了原计划 Day 3 最重的部分（几何距离模块）
2. barrier / CCD 的接口设计已在 Day 2 plan 中详细规定
3. 系统集成的插入点已明确（`compute_max_stepsize` + `alpha_max` + barrier 装配）

---

## Day 3 完成时必须成立的结果

### 功能结果

- tet block 落到地面后不会明显穿透
- 接近障碍物时 barrier energy 非零且增大
- line search 步长确实受 CCD 安全上界约束
- 多步运行不出现 NaN / inf
- `enable_contact=false` 时 Day 1 行为保持不变

### 代码结果

- `ObstacleBody` 是完整的静态三角网格
- `IPCSystem` 可以注册 obstacle 并处理接触
- contact 路径不破坏现有无接触路径
- `IPCSystem` 可通过 `PhysicsSystem` 接入主运行时
- 代码结构支持后续加 `ShellBody`

### 测试结果

- 标量 barrier FD 测试通过
- pair barrier gradient FD 测试通过
- CCD 安全步长测试通过
- 至少 1 个 contact smoke test 通过

### 交付物

- 编译通过的完整 contact pipeline
- 1 个可运行的 contact demo（ground contact）
- 已知限制说明
- 下一阶段待办列表

---

## Day 3 之后的状态

Day 3 结束后，5 天计划的核心交付全部完成。剩余的是**非核心优化和扩展**：

### 已完成

- TetBody + ObstacleBody
- implicit Euler + Newton
- barrier + CCD
- bunny/block contact demo
- PhysicsSystem 接入

### 明确未完成（known limitations）

- ShellBody 完整 membrane + bending
- cloth self-contact
- friction
- adaptive barrier stiffness 完整版
- 高性能 broad phase（BVH / spatial hash）
- 完整 hyperelastic 材料库
- coupling with RigidSystem

### 推荐下一步优先级

1. ShellBody 最小 membrane FEM
2. Shell bending
3. Self-contact
4. Friction
5. Coupling layer
6. 参数标定与 profiling
