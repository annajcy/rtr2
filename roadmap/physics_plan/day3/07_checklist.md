# Day 3 Checklist

## 执行顺序与进度追踪

按顺序执行，每完成一项打勾。

### Phase 1: 完成 Collision Mesh + Candidates

- [ ] `collision_mesh.hpp` 完成（obstacle 顶点支持、DOF 映射、统一 vertex_position 接口）
- [ ] `collision_candidates.hpp` 完成（PT/EE 候选生成 + 共享顶点/边/邻接过滤）
- [ ] collision mesh 构建测试通过
- [ ] 候选过滤测试通过

### Phase 2: Barrier Function + Potential

- [ ] `contact/barrier_function.hpp` 完成（标量 $b(s, \hat{s})$ 及一二阶导数）
- [ ] 标量 barrier gradient FD 测试通过
- [ ] 标量 barrier Hessian FD 测试通过
- [ ] `contact/barrier_potential.hpp` 完成（泛型 pair barrier + 链式法则 + PSD projection + 全局装配）
- [ ] PT pair barrier gradient FD 测试通过
- [ ] EE pair barrier gradient FD 测试通过
- [ ] obstacle 顶点不装配到全局 DOF 测试通过

### Phase 3: ObstacleBody 升级

- [ ] `model/obstacle_body.hpp` 升级（顶点 + 三角形 + 边 + build_edges）
- [ ] `generate_ground_plane()` 完成
- [ ] obstacle body 构建测试通过

### Phase 4: CCD + 安全步长

- [ ] `ccd/ccd_config.hpp` 完成
- [ ] `ccd/collision_free_stepsize.hpp` 完成（conservative backtracking + Distance concept 复用）
- [ ] CCD 安全步长测试通过：单对 PT 不允许穿透
- [ ] CCD 安全步长测试通过：单对 EE 不允许穿透
- [ ] CCD 安全步长测试通过：多对候选返回最小 alpha_max
- [ ] `line_search.hpp` 修改：新增 `alpha_max` 参数
- [ ] `newton_solver.hpp` 修改：`NewtonProblem` 新增 `compute_max_stepsize`

### Phase 5: IPCSystem Contact 集成

- [ ] `IPCConfig` 新增 contact 参数（`dhat_squared`, `barrier_stiffness`, `enable_contact`）
- [ ] `IPCSystem` 新增 obstacle body 管理接口
- [ ] `IPCSystem` 新增 collision mesh + candidates 成员
- [ ] `IPCSystem.step()` 中构建 collision mesh 和 candidates
- [ ] 总能量装配新增 barrier 贡献
- [ ] 总 gradient 装配新增 barrier 贡献
- [ ] 总 Hessian 装配新增 barrier 贡献
- [ ] Newton solver 集成 CCD 安全步长
- [ ] contact smoke test 通过：tet block 落到地面不穿透
- [ ] `enable_contact=false` 时行为与 Day 1 一致

### Phase 6: PhysicsSystem 接入 + Demo

- [ ] `IPCSystem` 接入 PhysicsSystem 主循环（最小侵入方式）
- [ ] `examples/editor/ipc_ground_contact_editor.cpp` 完成
- [ ] `examples/CMakeLists.txt` 新增 target
- [ ] demo 编译运行
- [ ] demo 中 block 落到地面后停住，不穿透
- [ ] demo 运行 10 秒无 NaN

### Phase 7: 收尾

- [ ] 所有新增测试通过
- [ ] 编译无 warning
- [ ] `enable_contact=false` 不破坏现有功能
- [ ] 已知限制清单编写（见下）
- [ ] 下一阶段待办编写（见下）

---

## Day 3 结束时必须回答的问题

1. barrier 在 $d \to 0$ 时是否 $\to +\infty$？在 $d = \hat{d}$ 时是否 $C^1$ 过渡到零？
2. CCD 安全步长是否真正约束了 line search？
3. tet block 能否在地面上停住？
4. 长时间运行（100+ 步）是否稳定？
5. `enable_contact=false` 是否保持 Day 1 行为？
6. 后续加 self-contact 时该往哪里插？

---

## Day 3 结束后输出物

- [ ] 编译通过的完整 contact pipeline（geometry + contact + CCD）
- [ ] 通过 FD 检查的 barrier 测试
- [ ] 1 个 contact demo 可运行
- [ ] 1 个 contact smoke test 通过
- [ ] 已知限制清单
- [ ] 下一阶段待办

---

## 已知限制（Day 3 结束时填写）

Day 3 交付的是"研究型核心 IPC 版本"，以下功能明确未完成：

- [ ] ShellBody / cloth membrane + bending
- [ ] Self-contact（deformable-deformable 同体接触）
- [ ] Friction
- [ ] Adaptive barrier stiffness（IPC 论文 Algorithm 1）
- [ ] 高性能 broad phase（BVH / spatial hash）
- [ ] 完整 hyperelastic 材料库（仅有 fixed corotated）
- [ ] Coupling with RigidSystem
- [ ] Kinematic obstacle（运动障碍物）

---

## 下一阶段优先级建议

1. **ShellBody 最小 membrane FEM** — 复用 Distance concept + barrier + CCD
2. **Shell bending** — discrete shell bending energy
3. **Self-contact** — collision mesh 放开同体非邻接候选
4. **Friction** — semi-implicit friction（IPC 论文 §4）
5. **Adaptive barrier stiffness** — 收敛后自动调 κ
6. **Coupling layer** — IPCSystem + RigidSystem 联合求解
7. **性能优化** — BVH broad phase、并行装配

---

## Day 3 输出文件清单

### 新建

| 文件 | 说明 |
|------|------|
| `src/rtr/system/physics/ipc/contact/barrier_function.hpp` | 标量 barrier |
| `src/rtr/system/physics/ipc/contact/barrier_potential.hpp` | barrier 全局装配 |
| `src/rtr/system/physics/ipc/ccd/ccd_config.hpp` | CCD 参数 |
| `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp` | conservative 安全步长 |
| `examples/editor/ipc_ground_contact_editor.cpp` | contact demo |
| `test/system/physics/ipc/contact/barrier_function_test.cpp` | barrier FD 测试 |
| `test/system/physics/ipc/contact/barrier_potential_test.cpp` | pair barrier 测试 |
| `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp` | CCD 测试 |
| `test/system/physics/ipc/ipc_contact_smoke_test.cpp` | 系统级 smoke test |

### 修改

| 文件 | 修改内容 |
|------|---------|
| `src/rtr/system/physics/ipc/contact/collision_mesh.hpp` | 完善 obstacle 支持 |
| `src/rtr/system/physics/ipc/contact/collision_candidates.hpp` | 完善候选过滤 |
| `src/rtr/system/physics/ipc/model/obstacle_body.hpp` | 完整实现 |
| `src/rtr/system/physics/ipc/core/ipc_system.hpp` | contact 集成 |
| `src/rtr/system/physics/ipc/solver/newton_solver.hpp` | CCD 回调 |
| `src/rtr/system/physics/ipc/solver/line_search.hpp` | alpha_max |
| `examples/CMakeLists.txt` | 新增 demo |
| `test/CMakeLists.txt` | 新增测试 |
