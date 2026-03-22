# Day 1 Checklist

## 执行顺序与进度追踪

按顺序执行，每完成一项打勾。

### Phase -1: 清理 Physics 目录

- [ ] `collision/` 移入 `rigid_body/collision/`
- [ ] 修改 `rigid_body_world.hpp`、`collider.hpp`、`contact.hpp` 的 collision include 路径
- [ ] `common/` 文件分流：ids/material/step_context -> `rigid_body/`
- [ ] `normal_recompute.hpp` 提到 `physics/` 顶层
- [ ] 删除 `cloth/` 全链路（源/框架/编辑器/测试/示例）
- [ ] 修改 `physics_system.hpp`：去掉 ClothWorld
- [ ] 修改 `scene_physics_step.hpp`：去掉 cloth sync
- [ ] 修改 `inspector_panel.hpp`：去掉 cloth include
- [ ] `fem/tet_surface_extract.hpp` 移到 `ipc/model/`
- [ ] 删除 `coupling/`、`common/`（如已空）
- [ ] 新建 `ipc/{core,model,energy,solver}/` 目录
- [ ] 更新 `test/CMakeLists.txt`：删 cloth 测试、改 collision 测试路径
- [ ] `cmake --build` 成功 + 现有测试通过

### Phase 0: Eigen 引入

- [ ] `conanfile.py` 加 `eigen/3.4.0`
- [ ] `src/CMakeLists.txt` 加 `Eigen3::Eigen`
- [ ] conan install + cmake build 成功
- [ ] 现有测试不受影响

### Phase 1: 数据结构

- [ ] 建目录 `src/rtr/system/physics/ipc/{core,model,energy,solver}/`
- [ ] 建目录 `test/system/physics/ipc/solver/`
- [ ] `core/ipc_state.hpp` 完成
- [ ] `model/ipc_body.hpp` 完成
- [ ] `model/tet_body.hpp` 完成（含 precompute + AxisConstraint + fix_vertex/fix_vertex_axis）
- [ ] `model/obstacle_body.hpp` 占位完成
- [ ] tet block 生成工具完成
- [ ] `ipc_state_test.cpp` 通过
- [ ] `tet_body_test.cpp` 通过

### Phase 2: 能量模块

- [ ] `energy/inertial_energy.hpp` 完成
- [ ] `energy/gravity_energy.hpp` 完成
- [ ] `energy/tet_material_model.hpp` concept 定义完成
- [ ] `energy/tet_fixed_corotated_energy.hpp` 完成（至少 energy + PK1），满足 `MaterialModel` concept
- [ ] `energy/material_energy.hpp` 完成（`MaterialModel` → 全局 DOF 的桥接）
- [ ] 单 tet energy 手算验证正确

### Phase 3: Solver

- [ ] `solver/line_search.hpp` 完成
- [ ] `solver/newton_solver.hpp` 完成
- [ ] Dirichlet 消元逻辑正确
- [ ] SimplicialLDLT 线性求解可用
- [ ] solver log 输出 iteration/energy/gradient_norm/alpha

### Phase 4: IPCSystem

- [ ] `core/ipc_system.hpp` 完成
- [ ] `initialize()` 正确设置全局状态
- [ ] `step()` 完成一次完整时间步
- [ ] `compute_x_hat()` 含重力修正
- [ ] 总能量/梯度/Hessian 装配路径打通

### Phase 5: 测试

- [ ] `ipc_tet_smoke_test.cpp` SingleStepNoNaN 通过
- [ ] `ipc_tet_smoke_test.cpp` MultiStepStable 通过
- [ ] `ipc_tet_smoke_test.cpp` ZeroGravityStationary 通过
- [ ] `ipc_tet_smoke_test.cpp` SlipDBCBottomPlane 通过
- [ ] cmake build + ctest 全部通过

### 可选

- [ ] tet elastic energy FD gradient check
- [ ] inertial energy FD gradient check
- [ ] headless demo 入口

## Day 1 结束时必须回答的问题

1. `IPCState` 是否统一管理所有 DOF？
2. `TetBody` 的 Dm_inv / rest_volume 正确吗？
3. 总能量装配路径是否打通？
4. Newton solver 是否真正调用了 energy/gradient/Hessian？
5. Dirichlet 是通过消元实现还是 hack？
6. 明天加 barrier 时该往哪个位置插？
7. smoke test 是否真的跑通了？

## Day 1 结束后输出物

- [ ] 编译通过的代码骨架
- [ ] 3+ 个通过的测试
- [ ] 已知问题列表
- [ ] Day 2 第一优先级事项
