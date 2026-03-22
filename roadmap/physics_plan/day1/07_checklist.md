# Day 1 Checklist

## 执行顺序与进度追踪

按顺序执行，每完成一项打勾。

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
- [ ] `model/tet_body.hpp` 完成（含 precompute）
- [ ] `model/obstacle_body.hpp` 占位完成
- [ ] tet block 生成工具完成
- [ ] `ipc_state_test.cpp` 通过
- [ ] `tet_body_test.cpp` 通过

### Phase 2: 能量模块

- [ ] `energy/inertial_energy.hpp` 完成
- [ ] `energy/tet_material_model.hpp` 抽象完成
- [ ] `energy/tet_fixed_corotated_energy.hpp` 完成（至少 energy + PK1）
- [ ] `energy/tet_elastic_assembler.hpp` 完成
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
