# Day 2 Overview

## 目标

在 Day 1 的无接触 tet FEM 隐式求解基础上，加入 **obstacle contact**：让 tet body 可以和静态障碍物（地面、球体等）发生无穿透接触。

**`TetBody + ObstacleBody -> contact geometry -> barrier energy -> collision-free line search -> stable contact`**

## Day 1 已有的基础

Day 1 已经完成了完整的无接触 IPC 骨架：

| 模块 | 文件 | 状态 |
|------|------|------|
| 统一状态 | `ipc_state.hpp` | 完成 |
| Tet body | `tet_body.hpp` | 完成（含 block generator、precompute、material variant） |
| 惯性能量 | `inertial_energy.hpp` | 完成 |
| 重力能量 | `gravity_energy.hpp` | 完成 |
| 弹性能量 | `material_energy.hpp` + `tet_fixed_corotated.hpp` | 完成 |
| Newton solver | `newton_solver.hpp` | 完成（含 Dirichlet 消元、正则化、line search） |
| Line search | `line_search.hpp` | 完成（Armijo backtracking） |
| IPCSystem | `ipc_system.hpp` | 完成（多 body、ID 管理、动态添加/删除、auto-rebuild） |
| Scene sync | `ipc_scene_sync.hpp` + `ipc_tet_component.hpp` | 完成 |
| Demo | `ipc_fixed_end_block_editor.cpp` | 完成（固定一端 cantilever） |
| 测试 | 12 个测试文件 | 完成 |

## Day 2 要做什么

Day 2 的核心是 **接触几何 + barrier + CCD + obstacle body**，对应 `ipc_plan.md` 中的 Phase 2 + Phase 3。

5-day plan 原本把这些拆成 Day 2（barrier 无 CCD）和 Day 3-4（加 CCD），但实际上：

1. Barrier 没有 CCD 约束 line search 就不安全——大步长更新可能跳过 barrier 区间直接穿透
2. 几何距离模块、barrier、CCD 三者代码量都不大，且高度内聚
3. 把它们放在一天做可以端到端验证，避免 Day 2 结束时处于"有 barrier 但会穿透"的不稳定中间态

因此 Day 2 合并做：**接触几何 + barrier + CCD + obstacle contact**。

## Day 2 不做什么

- self-contact（需要拓扑邻接过滤，留 Day 3）
- friction（留 Day 3+）
- Neo-Hookean 或其他 hyperelastic 材料（留 Day 3+）
- 高性能 broad phase（先 brute force，留后续优化）
- adaptive barrier stiffness（先用固定 $\kappa$）
- cloth / ShellBody

## 执行顺序

```text
Phase 1: 几何距离模块                    (~2 hr)
Phase 2: Collision mesh + Broad phase   (~1.5 hr)
Phase 3: Barrier potential              (~1.5 hr)
Phase 4: ObstacleBody                   (~1 hr)
Phase 5: CCD                            (~1.5 hr)
Phase 6: 接入 IPCSystem                 (~1.5 hr)
Phase 7: 测试与 Demo                    (~1.5 hr)
```

详见各阶段文件。

## 成功标准

- tet block 在地面上停住，不穿透
- tet block 与球体障碍物接触时被推开，不穿透
- barrier energy 在接近接触时显著增大
- CCD 约束 line search：即使 Newton 方向很激进也不穿透
- 几何距离和 barrier 的 gradient/Hessian 通过有限差分检查
- 多步运行不出 NaN

## 理论→代码对照表

| 理论概念 | Day 2 代码文件 | 对应内容 |
|----------|---------------|----------|
| Point-triangle distance | `geometry/point_triangle_distance.hpp` | $d^2_{PT}(p, t_0, t_1, t_2)$ 及梯度、Hessian |
| Edge-edge distance | `geometry/edge_edge_distance.hpp` | $d^2_{EE}(e_0, e_1, e_2, e_3)$ 及梯度、Hessian |
| Collision mesh | `contact/collision_mesh.hpp` | 表面顶点/边/三角形 + body 标记 |
| Broad phase | `contact/broad_phase.hpp` | AABB 候选对生成 |
| Contact candidates | `contact/collision_candidates.hpp` | PT/EE 候选集 |
| Barrier function | `contact/barrier_potential.hpp` | $b(d, \hat{d})$ 标量 barrier + 链式法则 |
| Obstacle body | `model/obstacle_body.hpp` | 静态三角网格障碍物 |
| Point-triangle CCD | `ccd/point_triangle_ccd.hpp` | 连续碰撞检测，安全步长 |
| Edge-edge CCD | `ccd/edge_edge_ccd.hpp` | 连续碰撞检测，安全步长 |
| Collision-free step | `ccd/collision_free_stepsize.hpp` | 全局安全步长聚合 |

## 文件依赖图

```text
geometry/point_triangle_distance.hpp
geometry/edge_edge_distance.hpp
    <- contact/collision_mesh.hpp
         <- contact/broad_phase.hpp
              <- contact/collision_candidates.hpp
    <- contact/barrier_potential.hpp
    <- ccd/point_triangle_ccd.hpp
    <- ccd/edge_edge_ccd.hpp
         <- ccd/collision_free_stepsize.hpp
              <- ipc_system.hpp (接入总能量装配 + line search)
model/obstacle_body.hpp
    <- ipc_system.hpp (注册障碍物)
```
