# RTR2 IPC 5-Day Implementation Plan

## 范围约定

5 天内如果要做“完整的 IPC 系统”，必须把目标收敛成**研究型核心版本**，而不是生产级全功能系统。

这份计划默认 5 天内要交付的是：

- 一个统一的 `IPCSystem`
- 支持 `TetBody`
- 支持 `ObstacleBody`
- 支持 implicit Euler
- 支持 hyperelastic / elastic tet energy 接口
- 支持 point-triangle / edge-edge 接触
- 支持 barrier
- 支持 CCD 约束 line search
- 支持最小可用的 Newton solver
- 支持 bunny/floor/sphere 或 clamp obstacle 类 demo

这 5 天内**不要求完全做完**：

- cloth 完整 shell bending
- self-contact 大规模稳健版本
- friction 完整版本
- 高性能 broad phase
- 所有材料模型
- 生产级 profiling / 并行优化

如果你坚持把这些都算进“完整”，这个计划会失真，最后只会变成半成品堆砌。

## 总体策略

5 天冲刺必须遵守下面的顺序：

1. 统一状态与求解框架
2. tet FEM 无接触隐式求解
3. 接触几何与 barrier
4. CCD + collision-free line search
5. 场景接入、demo、数值检查与收尾

不要反过来：

- 不要先做 cloth
- 不要先做 self-contact
- 不要先做 friction
- 不要先做刚体耦合

## Day 1: 搭骨架并跑通无接触 Tet IPC

### 当天目标

建立 `IPCSystem` 的最小骨架，完成无接触 tet FEM 的单步隐式求解闭环。

### 必做内容

- 新建 `src/rtr/system/physics/ipc/` 目录结构
- 引入 `Eigen`
- 定义统一状态：
  - `ipc_state.hpp`
  - `ipc_body.hpp`
  - `tet_body.hpp`
  - `obstacle_body.hpp`
- 定义最小求解流程：
  - `ipc_system.hpp`
  - `newton_solver.hpp`
  - `line_search.hpp`
- 实现无接触总能量：
  - inertial energy
  - gravity energy
  - tet elastic energy 接口
- 实现 `TetBody` 最小数据：
  - tet connectivity
  - `Dm`
  - `inv(Dm)`
  - rest volume
- 实现最小 scene write-back

### 推荐当日只做的材料模型

- fixed corotated tet energy

原因：

- 实现速度快
- 数值更稳
- 先验证框架再切 hyperelastic

### 当日交付要求

- 能构造一个最小 `IPCSystem`
- 能注册一个 `TetBody`
- 能完成一次 implicit Euler 下的 Newton 迭代流程
- 单 tetrahedron 或小 tet mesh 在无接触情况下能稳定落体/变形
- 有最小 smoke test

### 当日验收标准

- 不出现 NaN
- 能量/梯度/Hessian 装配流程打通
- scene 中 deformable 顶点能够回写

### 当日输出文件建议

- `src/rtr/system/physics/ipc/core/ipc_state.hpp`
- `src/rtr/system/physics/ipc/model/ipc_body.hpp`
- `src/rtr/system/physics/ipc/model/tet_body.hpp`
- `src/rtr/system/physics/ipc/energy/inertial_energy.hpp`
- `src/rtr/system/physics/ipc/energy/tet_fixed_corotated_energy.hpp`
- `src/rtr/system/physics/ipc/solver/newton_solver.hpp`
- `test/system/physics/ipc/solver/ipc_tet_smoke_test.cpp`

## Day 2: 做稳 Tet FEM 和 Hyperelastic 接口

### 当天目标

把无接触 tet 求解打稳，并把 hyperelastic 路线的接口补齐，为 bunny 拉伸/挤压做准备。

### 必做内容

- 完善 Newton solver：
  - 收敛判据
  - Hessian regularization
  - PSD projection
- 增加 Dirichlet / pinned / region handle 支持
- 增加 kinematic target boundary 支持
- 抽象 tet material interface：
  - `TetMaterialModel`
  - fixed corotated
  - Neo-Hookean 接口占位或最小实现
- 支持 tet surface extraction 接入 collision proxy builder
- 加入最小 bunny 体网格加载路径

### 当日交付要求

- `TetBody` 支持固定区域与受控拖拽区域
- 能做 bunny 的“固定一端，拉动另一端”的最小实验
- 能在无接触条件下稳定运行多个时间步
- material interface 足够支持后续切换到 hyperelastic

### 当日验收标准

- 单步到多步行为稳定
- 受控边界条件有效
- `TetBody` 的 surface proxy 构建可用

### 强制测试

- 单 tetrahedron gradient / Hessian 有限差分检查
- boundary condition 消元正确性测试
- 小 tet mesh 多步稳定性 smoke test

### 当日输出文件建议

- `src/rtr/system/physics/ipc/energy/tet_material_model.hpp`
- `src/rtr/system/physics/ipc/energy/tet_neo_hookean_energy.hpp`
- `src/rtr/system/physics/ipc/core/dirichlet_constraints.hpp`
- `src/rtr/system/physics/ipc/adapter/tet_surface_proxy_builder.hpp`
- `test/system/physics/ipc/energy/tet_energy_fd_test.cpp`

## Day 3: 接触几何、Barrier、Obstacle Contact

### 当天目标

加入障碍物接触，使 tet body 可以和 floor / sphere / box proxy 发生无穿透接触。

### 必做内容

- 实现几何距离模块：
  - point-triangle distance
  - edge-edge distance
  - 距离梯度
  - 距离 Hessian
- 实现最小 broad phase：
  - 先用 brute force 或简化 AABB pair
  - 只要正确，不要先优化
- 实现接触候选过滤：
  - 同体非法邻接过滤
  - 非法共享拓扑过滤
- 实现 barrier potential：
  - scalar barrier
  - chain rule 到全局 DOF
- 实现 `ObstacleBody`：
  - static / kinematic triangle proxy

### 当日交付要求

- tet vs floor 不穿透
- tet vs sphere/box obstacle 不穿透
- barrier 项进入总能量
- Newton 每步能考虑接触项

### 当日验收标准

- 接触后不会明显穿过障碍物
- barrier energy 在接近接触时显著增大
- 几何距离和导数有限差分检查通过基础 case

### 强制测试

- point-triangle gradient/Hessian fd test
- edge-edge gradient/Hessian fd test
- barrier fd test
- obstacle contact smoke test

### 当日输出文件建议

- `src/rtr/system/physics/ipc/geometry/point_triangle_distance.hpp`
- `src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp`
- `src/rtr/system/physics/ipc/contact/collision_candidates.hpp`
- `src/rtr/system/physics/ipc/contact/barrier_potential.hpp`
- `src/rtr/system/physics/ipc/model/obstacle_body.hpp`
- `test/system/physics/ipc/contact/barrier_fd_test.cpp`

## Day 4: CCD 与 Collision-Free Line Search

### 当天目标

让接触求解具备 IPC 的关键特征：每一步搜索都保持 collision-free。

### 必做内容

- 实现 point-triangle CCD
- 实现 edge-edge CCD
- 给定 `dx` 计算 conservative `alpha_max`
- 把 CCD 接入 line search
- 调整 Newton + line search 逻辑：
  - Armijo
  - `alpha <= alpha_max`
- 增加最小距离与接触诊断日志

### 当日交付要求

- 高速靠近障碍物时不会因大步 Newton update 穿透
- line search 会被 CCD 安全步长裁剪
- 有最小可运行的 collision-free contact demo

### 当日验收标准

- 即使搜索方向很激进，也不产生显著穿透
- 日志能输出最小距离和安全步长
- 接触求解比 Day 3 明显更稳

### 强制测试

- point-triangle CCD safety test
- edge-edge CCD safety test
- line search respects `alpha_max` test

### 当日输出文件建议

- `src/rtr/system/physics/ipc/ccd/point_triangle_ccd.hpp`
- `src/rtr/system/physics/ipc/ccd/edge_edge_ccd.hpp`
- `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp`
- `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp`

## Day 5: 系统收尾、Demo、最小完整交付

### 当天目标

把前四天的核心模块整合成一个“最小完整 IPC 系统”交付版本，并产出目标 demo。

### 必做内容

- 整理 `PhysicsSystem` 接口，把 `IPCSystem` 接入主运行时
- 完善场景同步：
  - `TetBody` scene sync
  - `ObstacleBody` scene sync
- 做最小 demo 场景：
  - bunny fixed + stretch
  - bunny squeeze / obstacle compression
- 做参数面板或最小配置接口：
  - time step
  - Newton iterations
  - `dhat`
  - barrier stiffness
  - material params
- 补齐核心 smoke tests
- 整理调试输出
- 写最小文档说明如何运行 demo

### 当日交付要求

- `IPCSystem` 能在主程序里运行
- 至少 2 个 demo 可复现
  - hyperelastic/elastic bunny stretch
  - bunny obstacle squeeze
- 代码结构清晰，后续能继续加 `ShellBody`
- 形成一个“研究型核心 IPC 版本”

### 当日验收标准

- demo 可运行
- 不出现明显穿透
- 不出现明显单元翻转
- 代码已经具备后续接入 cloth 的接口

### 强制输出

- 一个 demo 入口
- 一份已知限制说明
- 一份下一阶段待办

## 5 天结束时应明确未完成项

5 天后大概率仍未完成，且应明确记录：

- `ShellBody` 完整 membrane + bending
- cloth self-contact
- friction
- adaptive barrier stiffness 完整版
- 高性能 broad phase
- 完整 hyperelastic 材料库
- coupling with `RigidSystem`

这些不应伪装成“已完成”，而应作为 Day 5 交付的一部分写入 known limitations。

## 5 天后的推荐下一步

如果这 5 天计划执行成功，下一阶段顺序建议是：

1. `ShellBody` 最小 membrane FEM
2. shell bending
3. self-contact
4. friction
5. coupling layer
6. 参数标定与 profiling

## 每日硬性验收清单

### Day 1

- 有 `IPCSystem`
- 有 `TetBody`
- 无接触单步能跑

### Day 2

- boundary handle 能跑
- bunny 最小拉伸能跑
- tet material interface 已抽象

### Day 3

- obstacle contact 能跑
- barrier 已接入总能量
- fd tests 过核心 case

### Day 4

- CCD 能跑
- line search 受安全步长约束
- 高速接触不显著穿透

### Day 5

- demo 场景能运行
- 形成研究型最小完整 IPC 版本
- 明确列出未完成项与下一步

## 结论

5 天内的正确目标不是“做完所有 IPC 变体”，而是：

- 做完 `TetBody + ObstacleBody`
- 做完 implicit Euler + Newton
- 做完 barrier + CCD
- 做出 bunny stretch / squeeze demo
- 把 `ShellBody` 接口预留好

如果你按这份计划执行，5 天后你会得到一个真正可继续长大的 `IPCSystem`；如果你试图同时把 cloth、self-contact、friction、刚体耦合全部塞进去，最后只会得到一堆互相阻塞的半实现模块。

