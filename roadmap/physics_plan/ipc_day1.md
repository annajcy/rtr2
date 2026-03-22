# RTR2 IPC Day 1 Detailed Plan

## Day 1 目标

Day 1 的目标不是“做出完整 IPC”，而是：

- 建立 `IPCSystem` 的最小代码骨架
- 跑通无接触的 tet FEM 隐式单步求解闭环
- 把后续 4 天都要复用的抽象定稳
- 对核心理论形成足够清晰的实现心智模型

如果 Day 1 做得对，后面几天是在这个骨架上加 contact / barrier / CCD；如果 Day 1 做错，后面几天会不断返工。

## Day 1 交付要求

Day 1 结束时必须实际交付：

- 一个最小 `IPCSystem`
- 一个最小 `TetBody`
- 一个统一的全局状态表示
- 一个最小 Newton 求解器框架
- 一个无接触 tet implicit Euler 单步求解流程
- 一个 smoke test
- 一个最小 demo 或最小 headless 运行入口

## Day 1 要实现的闭环

Day 1 要打通的不是“代码骨架闭环”，而是一个真实可执行的物理 timestep 闭环：

**`TetBody -> implicit energy solve -> updated positions/velocities -> scene write-back -> next step`**

更具体地说，今天必须真的跑通下面这条链：

1. 构造一个最小 tet mesh
2. 建立 `TetBody` 的 rest state
3. 组装全局 `IPCState`
4. 计算 $\hat{x}$
5. 组装总能量
   - inertial
   - gravity
   - tet elastic
6. 用 Newton 求解新的 $x$
7. 更新 $v = (x_{t+1} - x_t) / h$
8. 回写顶点到 scene / deformable mesh
9. 重复进入下一 fixed step

Day 1 的目标不是：

- 只把文件和类建出来
- 只做一次 energy evaluation
- 只写公式，不形成 timestep loop

## Day 1 Demo Target

Day 1 最推荐的 demo 是：

**一个顶部固定一小块顶点的 tet block，在重力下自然下垂，并表现出轻微弹性变形。**

推荐这个 demo 的原因：

- 能体现内部弹性能
- 能体现边界条件处理
- 更接近后续 bunny stretch / squeeze
- 不需要 contact / barrier / CCD

Day 1 的最小可接受效果：

- 一个单 tetrahedron 或一个很小的 tet block
- 顶部一部分顶点固定
- 其余顶点在重力下稳定更新
- 每帧顶点都被回写到场景

Day 1 不追求的效果：

- 撞地
- obstacle contact
- self-contact
- cloth
- friction
- bunny 高质量大变形展示

明确不要求：

- cloth
- self-contact
- friction
- CCD
- production-grade broad phase
- 多材料完整支持

## Day 1 理论目标

今天必须真正搞懂以下 5 件事。

### 1. IPC 在 Day 1 还不是“碰撞系统”

Day 1 只做无接触版本，所以今天最重要的是理解：

- IPC 的主干首先是一个**隐式优化框架**
- contact / barrier / CCD 是后续往这个框架里加的项

也就是说，Day 1 的总问题其实是：

$$
x^{t+1} = \arg\min_x \frac{1}{2h^2}(x-\hat{x})^T M (x-\hat{x}) + \Psi(x)
$$

其中：

- $x$ 是当前步未知位形
- $\hat{x} = x_t + h v_t + h^2 M^{-1} f_{\text{ext}}$
- $M$ 是质量矩阵
- $\Psi(x)$ 是 tet 内部弹性能

今天先不要想 barrier.

### 2. 为什么用 implicit Euler

因为你的目标不是“快写一个能动的动画”，而是做后续能承接 IPC contact 的稳定主干。

显式积分的问题：

- 大步长不稳
- 高刚度材料容易爆
- 后续和 barrier/CCD 结合不自然

implicit Euler 的优势：

- 稳定性更好
- 自然转成能量最小化问题
- 与 Newton/line search/contact 势能兼容

### 3. 为什么先做 TetBody

因为 Day 1 要的是“标准 FEM 主干”。

TetBody 的优点：

- DOF 和物理意义清楚
- 体弹性能定义直接
- 为后续 hyperelastic bunny 做准备
- 比 shell bending 更适合作为第一天骨架目标

今天不碰 `ShellBody`，只预留接口。

### 3.1 为什么 Day 1 仍然需要材料模型

即使 Day 1 的 demo 只是“固定顶部一小块，在重力下下垂”，也仍然需要材料模型。

原因：

- 如果只有 inertial + gravity，而没有内部弹性能
- 那系统就只是一个没有恢复力的点集
- 不会表现出真正的弹性变形

因此 Day 1 必须有：

- deformation gradient $F$
- 某种 tet elastic energy
- 对应的 gradient / Hessian

但 Day 1 不需要完整材料系统，只需要：

- 一个最小 `TetMaterialModel` 抽象
- 一个具体实现：`FixedCorotatedTetMaterial`

推荐 Day 1 的材料层级：

- `TetMaterialModel`
  - `energy(...)`
  - `gradient(...)`
  - `hessian(...)`
- `FixedCorotatedTetMaterial`

### 4. Newton 求解的本质

今天要理解的不是“怎么把线性系统解出来”，而是：

- 你在最小化一个非线性能量
- 每次迭代都要组装：
  - energy
  - gradient
  - Hessian
- 用

$$
H \Delta x = -g
$$

得到搜索方向

然后用 line search 更新：

$$
x \leftarrow x + \alpha \Delta x
$$

Day 1 没有 CCD，所以 line search 先只做基础版。

### 5. DOF 视角

今天必须统一成一个思维方式：

- 所有求解变量都进全局 DOF 向量
- body 只是这个全局向量的一个切片

这会直接影响你之后写 `TetBody`、`ShellBody`、`ObstacleBody` 的方式。

今天的统一抽象要定成：

- `IPCState`
  - $x$
  - $\hat{x}$
  - $v$
  - $\text{mass\_diag}$
- `IPCBody`
  - dof offset
  - vertex count
  - material / topology / boundary metadata

### 6. Day 1 的边界条件本质上就是 Dirichlet

对于“固定顶部一小块 tet 顶点”这个 demo，Day 1 只需要 Dirichlet boundary condition。

不需要：

- contact
- rigid handle
- constraint projection
- Lagrange multiplier
- augmented Lagrangian

你只需要把顶部被固定的顶点视为：

- $x_{fixed}$ 已知
- 它们不是优化变量

真正求解的是：

$$
\min_{x_{free}} E(x_{free}, x_{fixed})
$$

Day 1 最推荐的实现方式是：

- 直接消元 constrained DOFs

也就是说：

- 给每个顶点的 `x/y/z` DOF 标记是否固定
- 组装线性系统时只对 free DOFs 求解
- 固定 DOFs 直接保持给定值

## Day 1 理论学习安排

建议用 2.5 小时做理论对齐，不要跳过。

### 09:00 - 09:40

主题：

- implicit Euler 的能量形式
- $\hat{x}$ 的物理意义
- 为什么可以把动力学写成优化问题

你要搞懂：

- 为什么惯性项是二次型
- 为什么这一步不是“先积速度再积位置”
- 为什么优化变量是 $x_{t+1}$

### 09:40 - 10:20

主题：

- tet FEM 的最小构件

你要搞懂：

- $Dm$
- $\text{inv}(Dm)$
- rest volume
- deformation gradient $F$
- 为什么轻微弹性变形依赖材料模型

Day 1 只需要理解固定 corotated 或最小弹性能需要哪些量，不需要把所有材料模型都啃完.

### 10:20 - 11:00

主题：

- Newton solver 的最小流程

你要搞懂：

- $E(x)$
- $g(x)$
- $H(x)$
- 搜索方向
- line search
- 为什么 Hessian regularization 迟早需要

### 11:00 - 11:30

主题：

- Day 1 的代码边界

你要明确：

- 今天哪些文件必须落地
- 哪些文件今天只建接口
- 哪些复杂项今天明确不做
- 顶部固定块的 Dirichlet 消元怎么落地

## Day 1 代码实践计划

建议按下面顺序写，不要跳步。

### Part 1: 建目录和核心类型

先建最小目录：

- `src/rtr/system/physics/ipc/core/`
- `src/rtr/system/physics/ipc/model/`
- `src/rtr/system/physics/ipc/energy/`
- `src/rtr/system/physics/ipc/solver/`
- `src/rtr/system/physics/ipc/adapter/`
- `test/system/physics/ipc/`

第一批文件：

- `core/ipc_state.hpp`
- `model/ipc_body.hpp`
- `model/tet_body.hpp`
- `model/obstacle_body.hpp`
- `energy/tet_material_model.hpp`
- `energy/tet_fixed_corotated_energy.hpp`
- `solver/newton_solver.hpp`
- `solver/line_search.hpp`
- `core/ipc_system.hpp`

这一阶段不要写太多细节，先把骨架立住。

### Part 2: 定义 IPCState

目标：

- 统一后续所有 body 的 DOF 管理

建议字段：

- `Eigen::VectorXd x`
- `Eigen::VectorXd x_prev`
- `Eigen::VectorXd v`
- `Eigen::VectorXd mass_diag`

建议接口：

- `resize_for_total_vertices(size_t n)`
- `position_segment(vertex_offset)`
- `velocity_segment(vertex_offset)`
- `dof_count()`

注意：

- Day 1 就要统一用 `Eigen`
- 不要在 solver core 里混入 `pbpt::math`

### Part 3: 定义 IPCBody / TetBody

`IPCBody` 建议只做抽象基类或轻量公共结构，不要一开始写复杂继承树。

最小需要：

- body type
- dof offset
- vertex count
- enabled flag

`TetBody` 最小需要：

- rest positions
- current-to-global vertex map
- tet connectivity
- $Dm$
- $\text{inv}(Dm)$
- rest volume
- density / material params
- fixed vertex / fixed DOF metadata

Day 1 只需要支持一个 `TetBody`。

### Part 4: 写 inertial + gravity energy

这是 Day 1 最容易先打通的一块。

建议单独写：

- `inertial_energy.hpp`
- `gravity_energy.hpp`

需要支持：

- energy
- gradient
- Hessian

惯性能量一定先写对，因为它决定了整个隐式框架能否成立。

### Part 5: 写 tet elastic energy 最小版本

Day 1 推荐：

- fixed corotated

不要一开始直接上最复杂的 hyperelastic。

最小要求：

- 单 tet energy
- 单 tet gradient
- 单 tet Hessian
- 支持装配到全局系统

如果 Hessian 太赶，至少保证结构已经预留，并明确 Day 1 先上可运行版本。

这里建议 Day 1 明确拆成：

- `TetMaterialModel` 抽象
- `FixedCorotatedTetMaterial` 具体实现

不要把 `TetBody` 和某一种材料硬绑定死。

### Part 6: 组装总能量

在 `IPCSystem` 里建立总能量装配路径：

- inertial
- gravity
- tet elastic

建议统一接口：

- `compute_energy(const IPCState&, ...)`
- `compute_gradient(...)`
- `compute_hessian(...)`

这里不要马上追求多态漂亮，先把路径打通。

### Part 7: 写最小 Newton Solver

Day 1 的 Newton solver 要求很克制：

- 计算总能量
- 计算总梯度
- 计算总 Hessian
- 解 $H \Delta x = -g$
- 做最小 line search
- 更新 $x$

线性求解器：

- Day 1 先用 `Eigen::SimplicialLDLT`

Day 1 的 line search：

- 先只做基础回溯
- 不做 CCD

### Part 8: scene write-back

目标：

- 让你能看到 tet 结果确实在动

最小方案：

- 从 `IPCState` 取出 `TetBody` 对应顶点
- 回写到 deformable mesh component

这里不求美，只求最小可见结果。

Day 1 推荐的具体视觉目标：

- 一个顶部固定的小 tet block
- 其余顶点在重力下下垂
- 画面上能看到稳定的轻微弹性变形

### Part 9: 测试

Day 1 最少要做这三个测试：

- `ipc_state_test`
  - DOF resize / segment 正确
- `tet_body_test`
  - `Dm` / `inv(Dm)` / rest volume 构建正确
- `ipc_tet_smoke_test`
  - 单 tetrahedron 或小 tet mesh 无接触单步求解可运行

如果还有时间，再做：

- inertial energy fd check
- 单 tet elastic energy fd check

## Day 1 时间切片建议

### 09:00 - 11:30

- 理论学习
- 定边界
- 画最小模块图

### 11:30 - 13:00

- 建目录
- 建 `IPCState`
- 建 `IPCBody` / `TetBody`

### 14:00 - 16:00

- 写 inertial / gravity
- 写 tet elastic 最小版本

### 16:00 - 18:00

- 写 `IPCSystem`
- 写 Newton solver
- 写 line search

### 19:00 - 21:00

- 写 smoke test
- 写最小 demo / write-back
- 修 NaN / 编译问题

### 21:00 - 22:00

- 清理接口
- 写 Day 1 总结
- 列出 Day 2 blocker

## Day 1 最小模块图

```text
IPCSystem
  -> IPCState
  -> TetBody
  -> EnergyAssembler
       -> InertialEnergy
       -> GravityEnergy
       -> TetElasticEnergy
  -> NewtonSolver
       -> LineSearch
       -> SparseLinearSolve
  -> SceneWriteback
```

## Day 1 必须避免的坑

### 1. 不要先做漂亮抽象

Day 1 的目标是把闭环跑通，不是设计满分架构。

不要一开始就：

- 设计过度复杂的继承体系
- 写一堆将来才会用到的接口

### 2. 不要先做 cloth

今天如果碰 cloth，基本就是自找阻塞。

### 3. 不要先做 contact

Day 1 根本不该碰 barrier / CCD。

### 3.1 不要把“自由下落/下垂”误当成不需要材料模型

如果你想看到“轻微弹性变形”，就必须有内部弹性能。

所以 Day 1 虽然不做完整材料系统，但一定要有最小材料抽象和一个 concrete model。

### 4. 不要在 solver core 混用数学类型

`pbpt::math` 留在 scene/render 边界。solver core 统一 `Eigen`。

### 5. 不要跳过 smoke test

没有 smoke test，Day 2 你会不知道骨架是不是从一开始就歪了。

## Day 1 完成后的自检问题

Day 1 结束前，你至少要能明确回答：

- `IPCState` 是否已经统一管理所有 DOF？
- `TetBody` 的 rest shape 数据是否齐全？
- `TetMaterialModel` 是否已经存在最小抽象？
- 总能量是否已经能统一装配？
- Newton solver 是否已经真正调用了 energy/gradient/Hessian？
- 顶部固定块是否通过 Dirichlet 消元而不是临时硬编码实现？
- 最小 demo 是否真的跑起来了？
- 明天加 contact 时，应该往哪个模块插 barrier？

如果其中任何一个答案是“不清楚”，Day 1 其实还没结束。

## Day 1 结束时的输出物

必须留下：

- 编译通过的最小代码骨架
- 一个 smoke test
- 一个最小 demo 或 headless 入口
- 一个 `Day 1 Notes`，记录：
  - 已完成
  - 未完成
  - 已知数值问题
  - Day 2 第一优先级

## Day 1 的成功标准

最短定义：

- 你已经有了一个**真正可执行的无接触 Tet IPC 核心骨架**

不是：

- 只有文档
- 只有类定义
- 只有公式草稿

如果到 Day 1 结束时，你能在运行里看到一个 tet body 在 implicit 框架下稳定更新，并且代码已经具备继续插入 barrier/CCD 的位置，那 Day 1 就是成功的。

## Day 1 File-by-File TODO Checklist

这一节的目标不是列“理想架构”，而是列出 Day 1 真正要落地的文件级任务。建议按顺序做，不要并行摊太开。

### 1. `src/rtr/system/physics/ipc/core/ipc_state.hpp`

TODO：

- 定义统一全局状态结构 `IPCState`
- 包含：
  - `Eigen::VectorXd x`
  - `Eigen::VectorXd x_prev`
  - `Eigen::VectorXd v`
  - `Eigen::VectorXd mass_diag`
- 提供最小接口：
  - `resize_for_total_vertices(size_t vertex_count)`
  - `dof_count()`
  - 顶点 DOF segment 访问
- 明确 1 个顶点对应 3 个 DOF

验收：

- 能正确为 `N` 个顶点分配 `3N` 大小向量
- 能正确切出某个顶点的 position / velocity segment

### 2. `src/rtr/system/physics/ipc/model/ipc_body.hpp`

TODO：

- 定义 `IPCBodyType`
  - `Tet`
  - `Shell`
  - `Obstacle`
- 定义最小公共 body 元数据
  - `dof_offset`
  - `vertex_count`
  - `enabled`

Day 1 范围：

- 不追求复杂继承体系
- 只要能给 `TetBody` 和未来 body 留统一入口

验收：

- `TetBody` 能复用这层公共信息

### 3. `src/rtr/system/physics/ipc/model/tet_body.hpp`

TODO：

- 定义 `TetBody`
- 包含：
  - rest positions
  - tet connectivity
  - `Dm`
  - `inv(Dm)`
  - rest volume
  - density / material params
  - fixed vertex 或 constrained DOF metadata
- 提供预计算接口：
  - 计算每个 tet 的 `Dm`
  - 计算 `inv(Dm)`
  - 计算 rest volume

Day 1 范围：

- 只支持单个 `TetBody`
- 不做 shell / obstacle 的复杂逻辑

验收：

- 单 tetrahedron 的 rest 数据预计算正确
- 小 tet mesh 的预计算不会出 NaN

### 4. `src/rtr/system/physics/ipc/model/obstacle_body.hpp`

TODO：

- 先建最小占位结构
- 支持：
  - static / kinematic 标记
  - 最小 proxy metadata

Day 1 范围：

- 只建接口和最小数据结构
- 不接入 contact 求解

验收：

- `IPCSystem` 中可以合法持有 `ObstacleBody` 类型定义

### 5. `src/rtr/system/physics/ipc/energy/tet_material_model.hpp`

TODO：

- 定义最小 `TetMaterialModel` 抽象
- 最低接口：
  - `energy(...)`
  - `gradient(...)`
  - `hessian(...)`

Day 1 范围：

- 不做材料注册表
- 不做插件式工厂

验收：

- `TetBody` 的弹性能不会和某个具体模型硬绑定

### 6. `src/rtr/system/physics/ipc/energy/inertial_energy.hpp`

TODO：

- 实现 implicit Euler 对应的惯性能量
- 实现：
  - energy
  - gradient
  - Hessian

建议：

- 明确使用 `hat_x`
- Hessian 直接得到质量相关二次项

验收：

- 对简单向量 case 结果维度正确
- 不出现 NaN

### 7. `src/rtr/system/physics/ipc/energy/gravity_energy.hpp`

TODO：

- 实现重力势能
- 实现：
  - energy
  - gradient
  - Hessian

备注：

- Hessian 应该为零或空贡献

验收：

- 重力方向和符号正确

### 8. `src/rtr/system/physics/ipc/energy/tet_fixed_corotated_energy.hpp`

TODO：

- 实现 Day 1 唯一 concrete material：
  - `FixedCorotatedTetMaterial`
- 支持：
  - 单 tet energy
  - 单 tet gradient
  - 单 tet Hessian
  - 装配到全局系统

Day 1 范围：

- 先做 fixed corotated
- 不急着把 Neo-Hookean 一起做完

验收：

- 单 tet 在小扰动下 energy 合理
- gradient / Hessian 维度正确

### 9. `src/rtr/system/physics/ipc/solver/line_search.hpp`

TODO：

- 实现最小回溯 line search
- 支持：
  - 给定 `x`、`dx`
  - 尝试 `alpha = 1, 1/2, 1/4, ...`
  - 比较能量下降

Day 1 范围：

- 不做 CCD 限制
- 不做复杂 Wolfe 条件

验收：

- 当 full step 不合适时，能自动缩步

### 10. `src/rtr/system/physics/ipc/solver/newton_solver.hpp`

TODO：

- 实现最小 Newton solver 主流程
- 支持：
  - 计算总 energy
  - 计算总 gradient
  - 计算总 Hessian
  - 解 `H dx = -g`
  - 调 line search
  - 更新 `x`

建议：

- 线性系统后端先用 `Eigen::SimplicialLDLT`
- 日志至少输出：
  - iteration
  - energy
  - gradient norm
  - step length

验收：

- 单步求解能收敛或至少向合理方向下降

### 11. `src/rtr/system/physics/ipc/core/ipc_system.hpp`

TODO：

- 定义最小 `IPCSystem`
- 支持：
  - 注册 `TetBody`
  - 持有 `IPCState`
  - 计算 `hat_x`
  - 调用总能量装配
  - 调用 Newton solver
  - 更新速度

Day 1 范围：

- 只支持无接触
- 只支持 `TetBody`

验收：

- `IPCSystem::step(dt)` 能真实更新状态

### 12. `src/rtr/system/physics/ipc/adapter/` 相关文件

TODO：

- 至少建一个最小 write-back 工具
- 功能：
  - 从 `IPCState` 取 `TetBody` 对应顶点
  - 转回 scene/render 使用的顶点数组

Day 1 范围：

- 不追求通用 adapter 架构
- 只要够支撑 demo

验收：

- 场景中能看到 tet 顶点更新

### 13. `test/system/physics/ipc/ipc_state_test.cpp`

TODO：

- 测 `IPCState` resize
- 测 3N DOF 组织
- 测 segment 访问

验收：

- 基础状态管理正确

### 14. `test/system/physics/ipc/tet_body_test.cpp`

TODO：

- 测 `Dm`
- 测 `inv(Dm)`
- 测 rest volume
- 测固定顶点/Dirichlet 元数据

验收：

- tet 预计算数据正确

### 15. `test/system/physics/ipc/solver/ipc_tet_smoke_test.cpp`

TODO：

- 构造一个最小 tet mesh
- 构造最小 `IPCSystem`
- 执行一次或几次 step
- 检查：
  - 没有 NaN
  - 顶点状态发生变化
  - 固定顶点保持固定

验收：

- Day 1 主闭环真正可运行

### 16. 可选 Demo 入口文件

TODO：

- 做一个最小可视化入口，或 headless demo
- 展示：
  - 顶部固定的小 tet block
  - 在重力下下垂

优先级：

- 如果时间不够，至少保住 smoke test
- 有时间再做视觉 demo

验收：

- 能肉眼确认 Day 1 的目标效果

## Day 1 文件级执行顺序

建议严格按下面顺序：

1. `ipc_state.hpp`
2. `ipc_body.hpp`
3. `tet_body.hpp`
4. `tet_material_model.hpp`
5. `inertial_energy.hpp`
6. `gravity_energy.hpp`
7. `tet_fixed_corotated_energy.hpp`
8. `line_search.hpp`
9. `newton_solver.hpp`
10. `ipc_system.hpp`
11. `ipc_state_test.cpp`
12. `tet_body_test.cpp`
13. `ipc_tet_smoke_test.cpp`
14. 最小 demo / write-back

## Day 1 文件级完成定义

当你说 Day 1 做完时，至少意味着：

- 上面列表里的核心文件已经存在
- 不是空壳占位，而是有最小可执行实现
- smoke test 能跑
- 固定顶部一小块的 tet demo 能工作，或者至少 headless smoke case 能工作
