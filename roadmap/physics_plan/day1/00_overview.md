# Day 1 Overview

## 目标

跑通无接触 tet FEM 隐式求解闭环：

**`TetBody -> implicit energy solve -> updated positions/velocities -> scene write-back -> next step`**

## Demo Target

一个顶部固定一小块顶点的程序生成 tet block（3x3x3），在重力下自然下垂并表现出弹性变形。

## 不做什么

- cloth / ShellBody
- contact / barrier / CCD
- friction
- self-contact
- 高性能 broad phase
- 完整 hyperelastic 材料库
- scene 级集成（不接入 PhysicsSystem 主循环）

## 交付物

1. 编译通过的 IPC 代码骨架
2. 3 个单元测试通过
3. 1 个 smoke test 验证多步求解稳定
4. 可选：headless demo 入口

## 执行顺序

```
Phase 0: Eigen 引入与构建验证          (~30 min)
Phase 1: 核心数据结构                   (~1.5 hr)
Phase 2: 能量模块                       (~2 hr)
Phase 3: Newton solver + line search    (~1.5 hr)
Phase 4: IPCSystem 组装                 (~1 hr)
Phase 5: 测试与验证                     (~1.5 hr)
```

详见各阶段文件。

## 文件依赖图

```
ipc_state.hpp
  <- ipc_body.hpp
       <- tet_body.hpp
       <- obstacle_body.hpp (占位)
  <- tet_material_model.hpp
       <- tet_fixed_corotated_energy.hpp
  <- inertial_energy.hpp
  <- gravity_energy.hpp
  <- line_search.hpp
       <- newton_solver.hpp
            <- ipc_system.hpp
```

## 成功标准

- `IPCState` 统一管理所有 DOF
- `TetBody` rest shape 预计算正确（Dm, inv(Dm), volume）
- 总能量装配路径打通（inertial + gravity + elastic）
- Newton solver 调用 energy/gradient/Hessian 并收敛
- Dirichlet 消元正确处理固定顶点
- 多步运行不出 NaN
- 明天加 contact 时知道往哪插 barrier
