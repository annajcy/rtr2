# RTR2 RigidSystem Long-Term Plan

## 目标

为 RTR2 单独建设一套面向**中大规模刚体碰撞**的 `RigidSystem`，作为长期演进方案，与 `IPCSystem` 并行存在，通过 `CouplingLayer` 与软体/布料系统交互，而不是强行纳入统一的 IPC 非线性求解器。

这套系统的核心目标是：

- 支持大量刚体同时运动、碰撞、堆叠、休眠
- 支持稳定的 resting contact 和 friction
- 支持高吞吐量 broad phase / narrow phase / solver
- 支持少量高速体的 CCD
- 支持与未来 `IPCSystem` 的单向/双向耦合

## 为什么单独做 RigidSystem

大规模 rigid body 的核心难点和 deformable IPC 不同：

- 刚体更依赖 persistent contact manifold
- 刚体更依赖 warm start
- 刚体更依赖 sleeping 和 island decomposition
- 刚体更依赖便宜、可扩展的约束求解器
- 刚体通常用 primitive / convex proxy，而不是高精表面网格

因此长期上更合理的拆分是：

- `IPCSystem`
  - hyperelastic tet
  - codimensional cloth
  - self-contact / obstacle contact
- `RigidSystem`
  - 大规模 rigid collisions
  - 堆叠、睡眠、约束、并行化
- `CouplingLayer`
  - rigid <-> deformable 交互

## 推荐总路线

推荐算法主线：

- broad phase
  - `Dynamic AABB Tree` 为默认实现
  - 后期扩展 `SAP / MBP`
- narrow phase
  - primitive analytic contact
  - convex 使用 `GJK + EPA`
  - contact manifold clipping
- solver
  - 第一版 `Sequential Impulse / PGS`
  - 中期升级 `TGS`
- 稳定性机制
  - persistent manifolds
  - warm start
  - sleeping
  - island decomposition
- CCD
  - 只对 bullet body / 高速体开启

这是典型的高性能 rigid engine 路线，适合你的“长期计划”目标。

## 顶层架构

推荐结构：

- `src/rtr/system/physics/rigid/`
- `src/rtr/system/physics/rigid/core/`
- `src/rtr/system/physics/rigid/broad_phase/`
- `src/rtr/system/physics/rigid/narrow_phase/`
- `src/rtr/system/physics/rigid/contact/`
- `src/rtr/system/physics/rigid/solver/`
- `src/rtr/system/physics/rigid/ccd/`
- `src/rtr/system/physics/rigid/debug/`
- `src/rtr/framework/component/physics/rigid/`

推荐职责：

- `core/`
  - 刚体状态
  - shape registry
  - body manager
  - world integration
- `broad_phase/`
  - dynamic tree
  - pair generation
  - collision filtering
- `narrow_phase/`
  - primitive contact generation
  - GJK/EPA
  - manifold construction
- `contact/`
  - persistent manifold cache
  - contact feature ids
  - material combine rules
- `solver/`
  - velocity solver
  - position stabilization
  - joints
  - island solver
- `ccd/`
  - sweep tests
  - TOI scheduling
  - speculative contacts
- `debug/`
  - contact visualization
  - manifold visualization
  - sleeping/island overlays

## 数据模型

### 1. RigidBodyState

每个刚体至少需要：

- type
  - static
  - kinematic
  - dynamic
- world transform
  - position
  - orientation
- velocities
  - linear velocity
  - angular velocity
- mass properties
  - mass
  - inverse mass
  - inertia tensor in body space
  - inverse inertia tensor
- accumulators
  - force
  - torque
- damping
  - linear damping
  - angular damping
- sleep state
  - awake / sleeping
  - sleep timer
- flags
  - enable CCD
  - enable gravity
  - can sleep

### 2. Shapes

长期不要默认用 triangle soup 直接跑 rigid。

推荐 shape 层级：

- sphere
- box
- capsule
- convex hull
- compound shape
- static triangle mesh

原则：

- dynamic body 优先使用 primitive / convex / compound
- static world 才允许大三角网格
- 非凸 dynamic mesh 通过 convex decomposition 离线转换为 compound convex

### 3. Fixtures / Colliders

每个 body 可以挂多个 collider：

- local transform
- shape handle
- friction
- restitution
- density
- collision layer / mask
- sensor flag

### 4. Contact 数据

推荐每个 manifold 存：

- body pair
- collider pair
- contact normal
- up to 2-4 contact points
- local anchors
- penetration depth
- accumulated normal impulse
- accumulated tangent impulse(s)
- feature ids
- last updated frame

有 persistent manifold，warm start 才有意义。

## 碰撞检测路线

### 1. Broad Phase

第一推荐：`Dynamic AABB Tree`

原因：

- 适合动态插入/删除/移动
- 适合游戏/交互场景
- 工程复杂度可控

需要支持：

- fat AABB
- proxy insert/remove/update
- overlap pair generation
- collision filtering
- query / ray cast / shape cast

后续扩展：

- `SAP`
  - 适合线性移动较多的场景
- `MBP`
  - 适合超大世界分区

### 2. Narrow Phase

推荐拆成三层：

- primitive analytic contact
  - sphere-sphere
  - sphere-plane
  - box-plane
  - capsule-plane
  - sphere-box
- convex collision
  - `GJK`
  - `EPA`
- manifold generation
  - clipping / feature-based manifold construction

建议：

- primitive 优先手写解析碰撞
- convex hull 统一走 GJK/EPA
- 不要一开始就支持所有 shape 对

### 3. Persistent Manifold

这是 large-scale rigid 最关键的结构之一。

需要实现：

- feature id 匹配
- manifold point refresh
- contact point 失效剔除
- 新接触点插入策略
- 点数上限管理

目标：

- 减少接触抖动
- 提高 solver warm start 效果
- 改善 stacking stability

## 约束求解器

### 1. 主推荐：Sequential Impulse / PGS

第一版最务实的方案：

- velocity-level contact solver
- normal impulse
- friction impulse
- restitution

优点：

- 简单
- 成熟
- 易调试
- 易扩展 joint

### 2. 中期升级：TGS

如果想进一步提升 stacking 稳定性，推荐在 PGS 跑通后升级 `TGS`。

收益：

- 对大时间步更稳
- 堆叠效果更好
- position drift 更小

顺序建议：

1. 先做 PGS
2. 做 warm start
3. 做 persistent manifold
4. 再做 TGS

### 3. 位置修正

推荐两层：

- 主体靠 velocity solver 解决
- 辅助位置稳定化
  - Baumgarte style bias 或 split impulse

要避免：

- 单纯暴力位置投影导致抖动
- restitution 与 penetration correction 相互打架

### 4. 摩擦

刚体摩擦建议按 solver 约束直接做：

- 计算切向基
- 求解切向冲量
- 用库仑摩擦锥近似

注意：

- stacking 的稳定性很依赖摩擦 warm start
- tangent basis 的一致性会影响抖动

## 岛分解与睡眠

### 1. Island Decomposition

必须做。

原因：

- 提高缓存局部性
- 支持并行
- 只更新活跃子图

图节点：

- dynamic bodies
- contacts
- joints

每帧流程：

- 从 awake body 出发做 DFS/BFS
- 构建 simulation island
- 每个 island 独立求解

### 2. Sleeping

必须做。

需要：

- 线速度阈值
- 角速度阈值
- 持续静止计时器
- 碰撞唤醒传播

目标：

- 大量静止物体不再浪费 solver 时间
- pile resting 场景性能大幅提升

## CCD 路线

### 1. 不要全局默认 CCD

大规模 rigid 中，全局 CCD 通常代价太高。

推荐：

- 默认离散碰撞
- 只对 bullet body / 高速体启用 CCD

### 2. 推荐 CCD 方案

第一版：

- sphere/capsule sweep
- convex cast
- TOI 事件驱动子步

第二版：

- speculative contacts
- 更完整的 shape cast 管线

### 3. TOI 调度

需要：

- 发现最早 TOI
- 子步推进到 TOI
- 只重解相关 body/island

否则：

- 性能会很差
- 高速体会拖慢全局

## Joint 系统

如果长期要做刚体场景，joint 应尽早预留接口。

推荐支持顺序：

- fixed joint
- distance joint
- hinge joint
- slider joint

实现方式：

- 与 contact 一样走约束求解器
- 共用 warm start / iteration 机制

## 与 IPCSystem 的耦合边界

不要一开始就做 fully-coupled monolithic solve。

推荐三阶段：

### Phase A: Rigid -> IPC 单向

- rigid body 作为 kinematic / moving obstacle 给 `IPCSystem`

### Phase B: IPC -> Rigid 单向

- 把 deformable contact force 聚合成 rigid body 外力/外力矩

### Phase C: 迭代式双向耦合

- rigid step
- ipc step
- exchange contact data
- repeat 1-2 次

这比强耦合一个大 nonlinear system 现实得多。

## 性能路线

### 1. 数据布局

长期建议：

- SoA 优先于 AoS
- bodies / colliders / contacts 分离存储
- 热数据与冷数据分离

### 2. 并行化优先级

推荐顺序：

1. broad phase pair generation
2. narrow phase batch contact generation
3. island-level parallel solve
4. debug/profiling 数据汇总

### 3. 几何代理策略

要尽早定规矩：

- dynamic complex mesh 不直接参与大规模碰撞
- 统一使用 convex proxy / compound proxy
- static scene 才允许 triangle mesh

否则 narrow phase 成本会失控。

## 调试与可视化

建议尽早做：

- broad phase AABB 可视化
- contact normals 可视化
- manifold points 可视化
- sleeping bodies 着色
- island id 着色
- CCD bullet path 可视化

建议日志：

- body count
- collider count
- broad phase pair count
- manifold count
- active contact point count
- awake island count
- solver iterations
- CCD body count

## 测试建议

建议新增：

- `test/system/physics/rigid/broad_phase/`
- `test/system/physics/rigid/narrow_phase/`
- `test/system/physics/rigid/contact/`
- `test/system/physics/rigid/solver/`
- `test/system/physics/rigid/ccd/`

关键测试：

- dynamic tree 插入/更新/查询正确性
- GJK/EPA 基本 case
- manifold persistence 正确性
- normal impulse / friction impulse 有限 case
- box stack 稳定性
- sleeping 唤醒规则
- bullet body CCD 防穿透

## 推荐实现阶段

### Phase 0: 底座

目标：

- body / shape / collider 数据结构
- rigid world tick
- 基础 debug draw

### Phase 1: Basic Collision

目标：

- dynamic AABB tree
- sphere / box / plane
- 基础 narrow phase
- contact manifold 数据结构

交付：

- 基础刚体自由落体和碰撞

### Phase 2: Basic Solver

目标：

- sequential impulse
- normal impulse
- friction
- restitution
- basic position stabilization

交付：

- box stack
- sphere pile

### Phase 3: Persistence

目标：

- persistent manifold
- warm start
- material combine rules

交付：

- 堆叠稳定性显著提高

### Phase 4: Islands + Sleeping

目标：

- island decomposition
- sleeping
- wake propagation

交付：

- 中大规模 pile 场景性能可接受

### Phase 5: Convex Pipeline

目标：

- convex hull support
- GJK/EPA
- convex manifold generation

交付：

- 复杂 rigid proxy 碰撞

### Phase 6: CCD

目标：

- bullet body
- TOI scheduling
- shape cast / sweep

交付：

- 高速刚体不穿透

### Phase 7: Coupling

目标：

- 和 `IPCSystem` 对接
- moving obstacle
- force feedback

交付：

- rigid + deformable demo

## 推荐首个长期演示目标

如果作为长期计划，我建议按下面的 demo 路线验证：

1. `10x10` box stack 稳定静止
2. 数百个 sphere/box 混合堆落
3. 少量 convex rigid 碰撞
4. bullet body 穿过狭缝不穿透
5. rigid obstacle 与 deformable bunny / cloth 交互

## 最终建议

你的长期物理架构建议固定成：

- `IPCSystem`
  - hyperelastic tet
  - shell / codimensional cloth
- `RigidSystem`
  - large-scale rigid collisions
  - joints
  - sleeping
  - island solve
- `CouplingLayer`
  - rigid/deformable exchange

如果只看 rigid 这一侧，最重要的不是某个“高级理论名字”，而是这几个工程要点能不能一起落地：

- 动态 broad phase
- persistent manifold
- warm start
- sleeping
- island decomposition
- 合理 CCD 策略

这几个做扎实了，`RigidSystem` 才会真正像一个能长期演进的系统。

## 参考方向

- Box2D 的 solver / dynamic tree / island / sleeping 设计思路
- PhysX 的 TGS、broad phase、rigid contact 工程路线
- MuJoCo 关于 convex collision、碰撞过滤和求解流程的文档化经验

