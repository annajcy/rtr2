# RTR2 CouplingLayer Plan

## 目标

为 RTR2 设计一层独立的 `CouplingLayer`，负责连接：

- `IPCSystem`
  - hyperelastic tet
  - codimensional / shell FEM cloth
- `RigidSystem`
  - large-scale rigid collisions

这层的目标不是把两套系统硬拼成一个 monolithic solver，而是提供一个**分阶段、可控、可扩展**的耦合框架：

- 第一阶段先做单向耦合
- 第二阶段做弱双向耦合
- 第三阶段再评估是否需要更强的迭代耦合

## 核心设计原则

### 1. 不追求一开始 fully-coupled solve

不要一开始就把 rigid 和 deformable 拼成一个大非线性系统统一牛顿求解。

原因：

- 实现复杂度极高
- 调试困难
- 刚体和 deformable 的时间推进方式不同
- large-scale rigid 的性能需求和 IPC 的高质量接触求解并不一致

推荐长期路线：

- `IPCSystem` 保持对 soft contact / self-contact / hyperelastic 的优势
- `RigidSystem` 保持对堆叠、休眠、吞吐量的优势
- `CouplingLayer` 只负责交换几何、运动学边界、接触力、代理信息

### 2. 先做稳定，再做强耦合

耦合层最容易出问题的不是“没有理论统一”，而是：

- 力传递不稳定
- 两边时间步不一致
- 接触抖动
- 循环依赖导致爆振

因此推荐顺序一定是：

1. rigid -> IPC 单向
2. IPC -> rigid 单向
3. staggered weak coupling
4. sub-iterations
5. 必要时再考虑更强耦合

### 3. 几何代理优先于原始高精几何

在 coupling 里，不要默认直接拿双方原始几何互相碰撞。

推荐：

- `RigidSystem` 给 `IPCSystem` 提供 obstacle proxy
- `IPCSystem` 给 `RigidSystem` 提供 coupling proxy / aggregate response

原因：

- 更稳定
- 更便于控制成本
- 更便于调试与参数标定

## 推荐架构

建议新增目录：

- `src/rtr/system/physics/coupling/`
- `src/rtr/system/physics/coupling/core/`
- `src/rtr/system/physics/coupling/proxy/`
- `src/rtr/system/physics/coupling/exchange/`
- `src/rtr/system/physics/coupling/policy/`
- `src/rtr/system/physics/coupling/debug/`

推荐职责：

- `core/`
  - coupling world
  - registration
  - step scheduler
- `proxy/`
  - rigid obstacle proxy
  - deformable proxy
  - sampled contact surface
- `exchange/`
  - kinematic boundary transfer
  - contact impulse / force transfer
  - torque accumulation
- `policy/`
  - one-way policy
  - two-way weak coupling policy
  - substep / subiteration policy
- `debug/`
  - proxy visualization
  - exchanged force visualization
  - coupling diagnostics

## CouplingLayer 需要管什么

建议这层只负责以下几件事：

- 谁和谁发生耦合
- 用什么 proxy 耦合
- 何时交换数据
- 交换什么数据
- 如何避免重复计入接触

不要让它承担：

- 刚体内部碰撞求解
- IPC 内部 barrier/self-contact 求解
- 大量 shape-specific 几何细节

它应该是**编排层 + 交换层**，不是第三套物理引擎。

## 推荐的耦合对象类型

### 1. Rigid As Obstacle

最先做，也最重要。

Rigid 给 IPC 提供：

- world-space obstacle surface proxy
- 当前位姿
- 当前速度场
- 碰撞材质

IPC 用这些作为：

- moving obstacle
- kinematic boundary
- codimensional / volumetric contact object

这是最自然、最稳定的第一阶段。

### 2. Deformable As Force Source

第二阶段再做。

IPC 给 Rigid 返回：

- 接触反力总和
- 对 rigid center of mass 的总力矩
- 可选的接触区域信息

Rigid 把这些当成外力/外力矩积到下一步。

### 3. Deformable As Collision Proxy

这一步要更谨慎。

Rigid 若要直接和 deformable 表面发生双向接触，需要 IPC 输出：

- surface proxy
- 接触点速度
- 法向/局部几何

但工程上不建议一开始让 rigid 直接把 deformable 当作普通 narrow-phase shape 使用。

更稳的做法是：

- 先由 IPC 负责 deformable-side contact
- 再把聚合反力反馈给 rigid

## 推荐的代理表示

### 1. Rigid -> IPC Proxy

优先使用：

- triangle obstacle mesh
- convex proxy
- primitive proxy

建议：

- static rigid 或慢速 rigid 可直接提供 world-space triangles
- dynamic rigid 优先提供低分辨率 proxy
- 不要把高面数 render mesh 直接送给 IPC

### 2. IPC -> Rigid Proxy

建议分两类：

- force proxy
  - 接触总力
  - 接触总力矩
- geometric proxy
  - surface sample points
  - local patch approximation

第一版只做 force proxy 就够了。

### 3. Handle Proxy

对于“固定、拉伸、挤压 bunny”这类场景，强烈建议引入 handle proxy：

- rigid tool / clamp / gripper
- region-based kinematic target
- deformable boundary attachment

也就是说，很多你表面上觉得是 rigid-deformable collision 的场景，实际上更适合建模为：

- rigid handle 驱动
- deformable 端 Dirichlet / kinematic boundary condition

这种方式比纯碰撞接触更稳，也更适合做受控展示。

## 三阶段耦合路线

### Phase A: 单向刚体障碍物

目标：

- rigid 只作为 moving obstacle 影响 IPC
- IPC 不把反力反馈给 rigid

每步流程：

1. `RigidSystem` 更新刚体位姿
2. `CouplingLayer` 生成 rigid obstacle proxies
3. `IPCSystem` 用这些 proxies 做接触求解
4. deformable 回写

适合：

- bunny 被 rigid tool 压缩
- cloth 落在 moving rigid obstacle 上

优点：

- 最稳
- 最容易实现
- 足够支持很多 demo

### Phase B: 单向反力反馈

目标：

- IPC 与 rigid 有弱双向交互
- deformable 接触反力传回 rigid

每步流程：

1. `RigidSystem` 先走一步预测
2. `CouplingLayer` 下发 obstacle proxies 给 `IPCSystem`
3. `IPCSystem` 解 deformable contact
4. `CouplingLayer` 汇总 deformable 对每个 rigid 的反力与力矩
5. 这些量在下一 rigid step 作为外力使用

优点：

- 保持系统解耦
- 已能产生“软体推动刚体”的效果

缺点：

- 反馈有一步延迟
- 强耦合场景下可能偏软

### Phase C: Staggered Weak Coupling

目标：

- 在一个 frame / fixed step 内做少量交替迭代

流程：

1. rigid predict
2. obstacle proxy to IPC
3. IPC solve
4. aggregate force back to rigid
5. rigid re-solve or correction
6. 重复 1-2 次

适合：

- soft body 明显推动 rigid
- rigid 与 deformable 相互挤压

建议：

- 一开始只做 1-2 次 sub-iteration
- 不要追求 fully converged partitioned coupling

## 具体交换的数据

### 1. 从 RigidSystem 到 IPCSystem

建议交换：

- rigid body id
- collider / proxy id
- world transform
- world-space proxy geometry
- rigid body velocity field
  - 线速度
  - 角速度
- friction / restitution-like material params
- enabled / sleeping 状态

其中速度场很重要，因为 IPC 对 moving obstacle 的接触/摩擦建模需要相对速度。

### 2. 从 IPCSystem 到 RigidSystem

建议交换：

- rigid body id
- total contact force
- total torque
- contact centroid
- optional contact patch metrics
  - area
  - average normal
  - max pressure proxy

第一版最小集合：

- `force`
- `torque`

### 3. 从 CouplingLayer 到 Scene / Debug

建议暴露：

- 当前 proxy 数量
- 当前发生耦合的 pair 数
- 每个 rigid 的耦合反力
- 每个 deformable 的耦合状态

## 接触责任划分

必须明确谁负责哪类接触，否则很容易重复求解。

推荐规则：

- rigid-rigid
  - 只由 `RigidSystem` 负责
- deformable-self
  - 只由 `IPCSystem` 负责
- deformable-obstacle
  - 默认由 `IPCSystem` 负责
- deformable-rigid
  - 第一版仍由 `IPCSystem` 负责 deformable 侧接触
  - `RigidSystem` 只接收聚合反馈

不要同时让两边都把同一对接触当主接触来解。

## 时间推进策略

### 1. 同 fixed dt，不同子步

推荐：

- 全局共享同一个 fixed dt
- `RigidSystem` 可以有自己的 solver iterations
- `IPCSystem` 可以有自己的 Newton / line search / barrier update
- `CouplingLayer` 只在固定同步点交换

### 2. Rigid 更快，IPC 更贵

实际中很常见：

- rigid 便宜
- IPC 贵

所以推荐：

- 默认每个 fixed step 只解一次 IPC
- rigid 可在其内部做更多 cheap iterations
- 只有强耦合场景才开启 coupling sub-iterations

### 3. 准静态模式

对于 bunny stretch / squeeze 这类展示型场景，推荐支持一个准静态模式：

- rigid handle 缓慢移动
- IPC 做准静态或强阻尼动态求解
- coupling 只把 handle 当作运动学边界

这通常比完全动态双向耦合更稳定，也更接近你想展示的材料响应。

## 对你目标场景的具体建议

### 1. Hyperelastic Stanford Bunny 被固定、拉伸、挤压

推荐优先建模为：

- bunny 在 `IPCSystem`
- clamp / tool 在 `RigidSystem`
- `CouplingLayer` 把 rigid clamp 转成：
  - moving obstacle
  - 或 region handle / kinematic target

更推荐：

- 固定和拉伸这类受控加载优先走 handle / boundary attachment
- 挤压和碰撞接触再走 obstacle contact

原因：

- 更稳定
- 更容易精确控制实验条件
- 更容易输出力-位移曲线

### 2. Cloth 与刚体交互

推荐：

- cloth self-contact 由 `IPCSystem` 处理
- rigid obstacle 由 `CouplingLayer` 下发 proxy
- 第一版不让 rigid 直接求 cloth 反馈
- 第二版再把布料反力聚合回 rigid

### 3. 大规模刚体场景 + 少量软体

推荐：

- rigid-rigid 仍完全留在 `RigidSystem`
- 只对和 soft body 临近的 rigid 开启 coupling
- 不要让所有 rigid 都参与 expensive deformable coupling

这一步对性能非常重要。

## 推荐核心模块

### 1. CouplingRegistry

负责：

- 注册哪些 rigid 与哪些 deformable 有耦合资格
- 配置耦合策略
- 配置 proxy 类型

### 2. ProxyBuilder

负责：

- 从 rigid 生成 obstacle proxy
- 从 deformable 生成 force proxy / geometric proxy

### 3. ForceAggregator

负责：

- 把 IPC 接触反力聚合到 rigid body
- 输出 force / torque

### 4. CouplingScheduler

负责：

- 控制 phase A / B / C 的执行顺序
- 控制 sub-iterations
- 控制同步点

### 5. CouplingDiagnostics

负责：

- 记录 exchanged force
- 记录 proxy 数量
- 记录耦合 pair
- 记录耦合稳定性问题

## 风险点

### 1. 双向耦合爆振

症状：

- rigid 和 deformable 互相推得越来越强
- 系统抖动或发散

规避：

- 从单向开始
- 加阻尼
- 限制反力反馈
- 限制 sub-iteration 次数

### 2. 几何代理过粗或过细

过粗：

- 接触不准
- 反力位置错误

过细：

- IPC 成本飙升

规避：

- coupling proxy 单独设计
- 不直接复用 render mesh

### 3. 重复计入接触

症状：

- 接触变得过硬
- 力翻倍

规避：

- 明确接触责任划分
- coupling 只做交换，不做第二次主求解

### 4. 时间步不协调

症状：

- rigid 已经穿过，IPC 还在上一步
- 反馈延迟明显

规避：

- 统一 fixed dt
- 只在同步点交换
- 必要时为强耦合对象启用 sub-iterations

## 测试建议

建议新增：

- `test/system/physics/coupling/registry_test.cpp`
- `test/system/physics/coupling/proxy_builder_test.cpp`
- `test/system/physics/coupling/force_aggregator_test.cpp`
- `test/system/physics/coupling/coupling_scheduler_test.cpp`

关键测试：

- rigid obstacle proxy 生成正确
- deformable 反力聚合为刚体 force/torque 正确
- 单向 coupling 不重复计入接触
- 双向 weak coupling 在简单 case 下稳定

## 推荐实施阶段

### Phase 0: 耦合边界定型

目标：

- 注册表
- proxy 数据结构
- 调度接口

### Phase 1: Rigid -> IPC 单向

目标：

- moving obstacle proxy
- rigid handle 驱动 deformable

交付：

- bunny 被 rigid tool 压缩
- cloth 与 moving box/sphere 交互

### Phase 2: IPC -> Rigid 反力反馈

目标：

- force / torque aggregation
- rigid 接收外力反馈

交付：

- soft body 推动轻刚体

### Phase 3: Weak Two-Way Coupling

目标：

- staggered iterations
- 稳定性控制

交付：

- 刚体与软体互相挤压的 demo

### Phase 4: 场景级策略

目标：

- 只耦合邻近对象
- coupling budget 控制
- selective sub-iterations

交付：

- 大场景下 coupling 成本可控

## 最终建议

对于 RTR2，我推荐把 `CouplingLayer` 定义成：

- 一个调度和交换层
- 一套 proxy 构建规则
- 一套力/力矩聚合规则
- 一套分阶段耦合策略

而不是：

- 第三套完整物理求解器
- rigid 和 IPC 的统一替代品

最重要的结论是：

- bunny 拉伸/夹持类场景优先用 handle / kinematic boundary
- 真正碰撞挤压再用 obstacle proxy
- 双向耦合要从弱耦合开始
- large-scale rigid 场景中只对少量必要对象启用 coupling

这样你的整体架构才会既能做研究型 demo，也能保持工程上可落地。

