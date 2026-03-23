# Day 2 Checklist

## 执行顺序与进度追踪

按顺序执行，每完成一项打勾。

### Phase 1: 几何距离模块

- [ ] 新建 `src/rtr/system/physics/ipc/geometry/` 目录
- [ ] `geometry/point_point_distance.hpp` 完成（距离平方 + gradient + Hessian）
- [ ] `geometry/point_edge_distance.hpp` 完成（距离平方 + gradient + Hessian）
- [ ] `geometry/point_triangle_distance.hpp` 完成（7 区域分类 + gradient + Hessian）
- [ ] `geometry/edge_edge_distance.hpp` 完成（含平行退化 + gradient + Hessian）
- [ ] PP gradient/Hessian FD 测试通过
- [ ] PE gradient/Hessian FD 测试通过
- [ ] PT gradient/Hessian FD 测试通过（多区域）
- [ ] EE gradient/Hessian FD 测试通过（含平行）

### Phase 2: Collision Mesh + Broad Phase

- [ ] 新建 `src/rtr/system/physics/ipc/contact/` 目录
- [ ] `contact/collision_mesh.hpp` 完成（合并 tet 表面 + obstacle 表面、边提取、body 标记）
- [ ] `contact/collision_candidates.hpp` 完成（brute force PT/EE 候选 + 共享顶点/边过滤 + 同体过滤）
- [ ] collision mesh 构建测试通过
- [ ] 候选过滤测试通过

### Phase 3: Barrier Potential

- [ ] `contact/barrier_function.hpp` 完成（标量 $b(s, \hat{s})$ 及一二阶导数）
- [ ] `contact/barrier_potential.hpp` 完成（链式法则装配到全局 DOF + PSD projection）
- [ ] 标量 barrier FD 测试通过
- [ ] 单对 barrier gradient FD 测试通过

### Phase 4: ObstacleBody

- [ ] `model/obstacle_body.hpp` 升级（三角网格 + 边 + generator）
- [ ] `generate_ground_plane()` 完成
- [ ] `collision_mesh.hpp` 更新：支持 obstacle 顶点（不在全局 DOF 中）
- [ ] obstacle body 构建测试通过

### Phase 5: CCD

- [ ] 新建 `src/rtr/system/physics/ipc/ccd/` 目录
- [ ] `ccd/collision_free_stepsize.hpp` 完成（conservative 安全步长）
- [ ] CCD 安全步长测试通过

### Phase 6: IPCSystem 集成

- [ ] `IPCConfig` 新增接触参数（`dhat_squared`, `barrier_stiffness`, `enable_contact`）
- [ ] `IPCSystem` 新增 obstacle body 管理（`create_obstacle_body`, `remove_obstacle_body`）
- [ ] `IPCSystem` 新增 collision mesh + candidates 成员
- [ ] `IPCSystem.step()` 修改：barrier 进入总能量装配
- [ ] `NewtonProblem` 新增 `compute_max_stepsize` 回调
- [ ] `line_search.hpp` 新增 `alpha_max` 参数
- [ ] `newton_solver.hpp` 集成 CCD 安全步长
- [ ] contact smoke test 通过：tet block 落到地面不穿透

### Phase 7: Demo

- [ ] `examples/editor/ipc_ground_contact_editor.cpp` 完成
- [ ] `examples/CMakeLists.txt` 新增 target
- [ ] demo 编译运行
- [ ] demo 中 block 落到地面后停住，不穿透
- [ ] demo 运行 10 秒无 NaN

### 可选

- [ ] 球体 obstacle generator
- [ ] 球体 contact demo
- [ ] adaptive barrier stiffness

## Day 2 结束时必须回答的问题

1. 几何距离函数在所有区域的梯度/Hessian 是否正确？
2. barrier 在 $d \to 0$ 和 $d = \hat{d}$ 的边界行为是否正确？
3. CCD 是否约束了 line search？
4. tet block 能否在地面上停住？
5. Day 3 加 self-contact 时该往哪里插？

## Day 2 结束后输出物

- [ ] 编译通过的 geometry + contact + CCD 模块
- [ ] FD 检查通过的测试
- [ ] 1 个 contact demo 可运行
- [ ] 已知问题列表
- [ ] Day 3 第一优先级事项

## Day 2 输出文件清单

### 新建

| 文件 | 说明 |
|------|------|
| `src/rtr/system/physics/ipc/geometry/point_point_distance.hpp` | PP 距离 |
| `src/rtr/system/physics/ipc/geometry/point_edge_distance.hpp` | PE 距离 |
| `src/rtr/system/physics/ipc/geometry/point_triangle_distance.hpp` | PT 距离 |
| `src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp` | EE 距离 |
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | 统一碰撞几何 |
| `src/rtr/system/physics/ipc/contact/collision_candidates.hpp` | 候选对生成 + 过滤 |
| `src/rtr/system/physics/ipc/contact/barrier_function.hpp` | 标量 barrier |
| `src/rtr/system/physics/ipc/contact/barrier_potential.hpp` | barrier 装配 |
| `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp` | CCD 安全步长 |
| `examples/editor/ipc_ground_contact_editor.cpp` | contact demo |
| `test/system/physics/ipc/geometry/point_point_distance_test.cpp` | |
| `test/system/physics/ipc/geometry/point_edge_distance_test.cpp` | |
| `test/system/physics/ipc/geometry/point_triangle_distance_test.cpp` | |
| `test/system/physics/ipc/geometry/edge_edge_distance_test.cpp` | |
| `test/system/physics/ipc/contact/barrier_function_test.cpp` | |
| `test/system/physics/ipc/contact/barrier_potential_test.cpp` | |
| `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp` | |
| `test/system/physics/ipc/ipc_contact_smoke_test.cpp` | |

### 修改

| 文件 | 修改内容 |
|------|---------|
| `src/rtr/system/physics/ipc/core/ipc_system.hpp` | obstacle 管理、barrier 装配、CCD 集成 |
| `src/rtr/system/physics/ipc/model/obstacle_body.hpp` | 从占位升级为完整实现 |
| `src/rtr/system/physics/ipc/solver/newton_solver.hpp` | `NewtonProblem` 新增 CCD 回调 |
| `src/rtr/system/physics/ipc/solver/line_search.hpp` | 新增 `alpha_max` 参数 |
| `examples/CMakeLists.txt` | 新增 demo target |
| `test/CMakeLists.txt` | 新增测试目标 |
