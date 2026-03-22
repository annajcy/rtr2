# RTR2 IPC Plan From Scratch

## 目标

把 RTR2 当前的物理系统升级为一套**完全从零实现**的 IPC 框架，核心目标是：

- 面向可变形体的大变形动力学
- 接触过程中无穿透
- 接触过程中无单元翻转
- 支持 cloth / tet FEM / kinematic obstacle
- 后续可扩展到 self-contact、friction、rigid coupling

这里的 IPC 指的是原始论文中的核心思想，而不是接某个现成库：

- 用**隐式时间积分**把动力学写成优化问题
- 用**barrier potential**处理正常接触
- 用**continuous collision detection (CCD)** 约束线搜索步长
- 用**incremental potential**在每个时间步里解一个非线性最优化问题

参考论文：

- [Incremental Potential Contact: Intersection- and Inversion-free, Large-Deformation Dynamics](https://cims.nyu.edu/gcl/papers/2020-IPC.pdf)

## 对当前仓库的判断

当前仓库已经有一些可复用边界，但还没有 IPC 需要的统一求解骨架：

- [`src/rtr/system/physics/physics_system.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/physics_system.hpp)
  - 目前只是顺序驱动 `RigidBodyWorld` 和 `ClothWorld`
- [`src/rtr/system/physics/cloth/cloth_world.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/cloth/cloth_world.hpp)
  - 当前是显式质量-弹簧，不是隐式优化求解
- [`src/rtr/framework/integration/physics/cloth_scene_sync.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/framework/integration/physics/cloth_scene_sync.hpp)
  - 已经有 scene 和 deformable mesh 的同步边界
- [`src/rtr/system/physics/fem/tet_surface_extract.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/fem/tet_surface_extract.hpp)
  - 已经有四面体表面提取逻辑，可作为 collision proxy 生成起点

这意味着最合理的路线不是继续修补现有 `ClothWorld`，而是新增一个统一的 `IPCSystem`。

## 顶层架构建议

### 总体结构

建议长期结构如下：

- `PhysicsSystem`
  - `RigidBodyWorld`：保留现有刚体系统，先不重写
  - `IPCSystem`：新建，统一管理所有可变形体和接触

### 为什么不要继续分 `ClothWorld` / `FemWorld`

在 IPC 框架下，cloth 和 tet FEM 共享的不是“材质模型”，而是以下基础设施：

- 状态向量组织
- 隐式时间积分
- 总能量装配
- 接触几何查询
- barrier 势能
- CCD
- Newton 求解
- 线搜索
- 稀疏线性系统

如果把 cloth 和 FEM 拆成两套 world，各写一套 solver，后面会重复实现几乎所有最难的部分。

### 标准 FEM 主干 + Codimensional 扩展

对你的 `IPCSystem`，推荐明确走下面这条路线：

- `3D deformable object`
  - 标准 volumetric FEM
  - 四面体网格
  - hyperelastic / elastic energy
- `cloth`
  - codimensional / shell FEM
  - 三角形表面网格
  - membrane + bending energy

也就是说：

- `IPCSystem` 统一的是求解框架
- 不统一的是单元类型和内部能量

因此正确的理解不是“所有对象都做成同一种 FEM”，而是：

- 统一 IPC 求解主干
- 在主干下支持多种 body model

推荐统一抽象：

- `IPCBody`
  - 通用 body 接口
- `TetBody`
  - volumetric FEM body
- `ShellBody`
  - codimensional / shell FEM cloth body
- `ObstacleBody`
  - static / kinematic obstacle body

这套抽象下：

- `TetBody` 和 `ShellBody` 共用状态管理、接触、CCD、barrier、Newton、line search
- `TetBody` 和 `ShellBody` 各自实现不同的内部能量装配

### 推荐目录

- `src/rtr/system/physics/ipc/`
- `src/rtr/system/physics/ipc/core/`
- `src/rtr/system/physics/ipc/model/`
- `src/rtr/system/physics/ipc/geometry/`
- `src/rtr/system/physics/ipc/energy/`
- `src/rtr/system/physics/ipc/contact/`
- `src/rtr/system/physics/ipc/ccd/`
- `src/rtr/system/physics/ipc/solver/`
- `src/rtr/system/physics/ipc/adapter/`
- `src/rtr/framework/component/physics/ipc/`

## 数学与数据表示

### 数值类型边界

强烈建议：

- scene/render/editor 边界继续使用 `pbpt::math`
- IPC 求解器内部统一使用 `Eigen`

推荐内部使用：

- `Eigen::VectorXd`
- `Eigen::MatrixXd`
- `Eigen::SparseMatrix<double>`
- `Eigen::Triplet<double>`

不要在能量装配核心路径里混用 `pbpt::math` 和 `Eigen`。这会让 Jacobian/Hessian 的实现和调试都变得很差。

### 统一状态表示

建议所有可变形对象都落到统一的全局 DOF 向量：

- `x`: 当前位形，大小 `3N`
- `x_prev`: 上一时间步位形
- `v`: 当前速度，大小 `3N`
- `m`: 质量向量或质量矩阵对角

对于每个 body，需要提供：

- 顶点到全局自由度区间的映射
- 本体的拓扑信息
- 本体的材料参数
- 本体的约束信息
- 本体的 collision proxy 信息

### 推荐核心数据结构

- `ipc_state.hpp`
  - `x`
  - `x_prev`
  - `v`
  - `mass_diag`
- `ipc_body.hpp`
  - body 类型
  - 顶点数量
  - dof offset
  - material params
  - collision proxy
- `tet_body.hpp`
  - tets
  - volumetric material
  - rest volume / `Dm` / `inv(Dm)`
- `shell_body.hpp`
  - triangles
  - edges
  - membrane params
  - bending params
- `obstacle_body.hpp`
  - static / kinematic collision proxy
- `collision_mesh.hpp`
  - 顶点
  - 边
  - 三角形
  - body / codim / boundary 标记
- `collision_candidates.hpp`
  - point-triangle 候选
  - edge-edge 候选
- `collision_constraints.hpp`
  - 激活接触约束
  - 最小距离
  - 法向相关数据

## 需要自己实现的核心模块

### 1. 几何与距离模块

这是 IPC 的地基，必须单独做好。

需要实现：

- vertex-vertex 距离
- point-edge 距离
- point-triangle 距离
- edge-edge 距离
- 对应的：
  - 距离平方
  - 梯度
  - Hessian

需要的能力：

- 返回最近点参数
- 处理退化情况
- 统一 DOF 排布下的局部导数装配
- 数值稳定，不出现 NaN

建议：

- 先统一写成“局部几何函数 + 局部导数”
- 再写“局部导数散装配到全局梯度/Hessian”

### 2. Broad Phase

从零实现时，不要一开始就做复杂 BVH 动态树。先做一个足够正确的 broad phase。

推荐路线：

- 第一版：AABB sweep-and-prune 或 uniform grid
- 第二版：BVH / spatial hash 优化

需要支持：

- point-triangle 候选生成
- edge-edge 候选生成
- 排除共享拓扑邻接的非法接触对
- 区分 self-contact 与 body-body contact

### 3. Continuous Collision Detection

CCD 是 IPC 的关键，没有它就没有可靠的 collision-free line search。

需要实现：

- 对 point-triangle 和 edge-edge 的连续相交检测
- 给定搜索方向 `dx`，计算允许步长 `alpha_max`
- 支持 conservative 结果，允许偏保守，不允许漏检

推荐策略：

- 第一版先做 conservative CCD
- 优先保证不会穿透
- 性能优化放后面

最小目标：

- 给定 `x` 和 trial step `x + alpha dx`
- 能找到一个使所有 primitive pair 都不相交的最大安全缩放

### 4. Barrier Potential

正常接触的核心是 barrier 势能。

你需要实现：

- 激活距离阈值 `dhat`
- 接触距离函数 `d(x)`
- barrier energy `b(d)`
- 对 `x` 的梯度
- 对 `x` 的 Hessian

实现要求：

- 当 `d` 接近 0 时势能迅速增大
- 当 `d >= dhat` 时该接触项为 0
- 梯度方向正确
- Hessian 数值稳定

这里建议写成两层：

- 标量 barrier 函数及其一二阶导数
- 通过链式法则映射到几何距离对 DOF 的梯度和 Hessian

### 5. Friction Potential

摩擦不要第一版就做，但架构必须预留。

后续需要实现：

- 接触切空间基
- lagged displacement / lagged velocity
- friction dissipative potential
- 对全局 DOF 的梯度和 Hessian 近似

建议：

- 第一阶段 friction 关闭
- 第二阶段先做 lagged friction

### 6. 弹性能模块

这是 cloth 和 tet FEM 的差异部分。

cloth 推荐明确走 **codimensional / shell FEM** 路线，而不是 mass-spring：

- membrane energy
  - 基于三角形表面单元的 2D FEM / shell FEM
  - 可从三角形 StVK 或 fixed corotated membrane 开始
- bending energy
  - hinge-based discrete bending
  - 这部分属于离散壳弯曲能，不是 tet FEM

也就是说，这个 plan 里的 cloth 是：

- 以三角形表面单元为离散对象
- 以膜能 + 弯曲能为内部能量
- 通过 codimensional contact 进入同一套 IPC 求解器

而不是：

- 当前仓库里的显式 mass-spring cloth
- 也不是体四面体的 tet FEM

因此对你的问题，一个明确结论是：

- `3D 物体` 推荐直接走标准 FEM 版本
- `cloth` 不走 tet FEM，而走 codimensional shell FEM
- 两者放在同一个 `IPCSystem` 中
- 统一 contact/solver，分离 internal energy

tet FEM 推荐：

- fixed corotated 先落地
- 然后扩展到真正的 hyperelastic model
  - compressible Neo-Hookean
  - stable Neo-Hookean / invertible hyperelastic variant

每种能量都要实现：

- energy
- gradient
- Hessian

不要跳过 Hessian。IPC 的稳定性和收敛性很依赖二阶信息。

### 7. 惯性能量与时间积分

建议第一版用 implicit Euler。

每个时间步的优化问题写成：

$$
x^{t+1} = \arg\min_x \frac{1}{2h^2}(x-\hat{x})^T M (x-\hat{x}) + \Psi(x) + \kappa B(x)
$$

其中：

- $\hat{x} = x_t + h v_t + h^2 M^{-1} f_ext$
- $\Psi(x)$ 是内部弹性能
- $B(x)$ 是 barrier contact energy

如果暂时不开 friction，这个目标已经足够支撑第一阶段系统。

### 8. Dirichlet / Pinned 约束

不要一开始用大罚项。

推荐顺序：

- 第一版：直接消元固定自由度
- 第二版：支持一般 kinematic boundary condition

需要实现：

- 从全局线性系统里消去固定 DOF
- 更新牛顿步和梯度时正确处理边界
- 兼容 cloth pinned vertices 和 tet anchor vertices

## 求解器设计

### 总体流程

每个 fixed step 建议按下面的顺序：

1. 从当前状态构造 $\hat{x}$
2. 初始化未知量 $x = x_t$
3. Newton 迭代
4. 更新速度 $v_{t+1} = (x_{t+1} - x_t) / h$
5. 回写 scene/render

### 每次 Newton 迭代

1. 根据当前 $x$ 更新 broad phase 候选
2. 构造激活接触约束集合
3. 装配总能量 $E(x)$
4. 装配总梯度 $g$
5. 装配总 Hessian $H$
6. 对 $H$ 做 PSD 投影或 regularization
7. 解 $H dx = -g$
8. 用 CCD 求最大安全步长 $\alpha_{\max}$
9. 在 $[0, \alpha_{\max}]$ 上做 Armijo line search
10. 更新 $x \leftarrow x + \alpha dx$

### Hessian 处理

推荐第一版支持两类稳定化：

- 对单元能量 Hessian 做 PSD projection
- 对全局矩阵加对角 regularization

如果不做这一步，Newton 方向会很不稳定。

### 线性求解器

建议封装接口：

- `SparseLinearSolver`

第一版后端：

- `Eigen::SimplicialLDLT`

后续可扩展：

- CHOLMOD
- Pardiso

但当前从零实现阶段，不要先把精力花在线性代数后端上。

### 收敛判据

建议同时看：

- $\||g||_inf$
- $\||dx||_inf$
- 相对位移 $\||dx|| / bbox_diag$
- 能量下降量
- line search 是否退化到极小步长

## 推荐的对象建模

### 1. Cloth

cloth 的长期目标不是保留现有 mass-spring，而是纳入统一 IPC 求解。

这里的 cloth 应明确理解为 **codimensional FEM / shell FEM cloth**：

- 几何上是 2D manifold embedded in 3D
- 力学上是表面单元的膜能 + 弯曲能
- 接触上走 point-triangle / edge-edge 的 codimensional IPC 管线

cloth 的 DOF 与接触几何的关系：

- 求解自由度就是表面顶点
- 接触几何也是这组表面顶点形成的边/三角形
- 因此 contact contribution 可以直接装配到 cloth 的全局 DOF

建议 cloth body 包含：

- rest positions
- triangles
- edges
- masses
- pinned mask
- membrane params
- bending params

碰撞类型：

- cloth vs obstacle
- cloth self-contact
- cloth vs tet surface

### 2. Tet FEM

这是最适合作为第一批核心对象的模型。

建议 tet body 包含：

- rest positions
- tet connectivity
- surface triangles
- surface edges
- $Dm$
- $\text{inv}(Dm)$
- rest volume
- density / Young / Poisson

tet 的 DOF 与接触几何的关系：

- 求解自由度是所有体顶点
- 接触通常只发生在表面三角形/边上
- 因此需要 surface contact contribution -> volumetric DOF 的映射

这也是 tet 和 cloth 在统一 IPC 框架下最重要的结构差异之一：

- cloth：surface DOF = collision DOF
- tet：collision DOF 是 volumetric DOF 的表面子集

如果你的最终成品是 hyperelastic Stanford bunny 被固定、拉伸、挤压，这里还必须额外支持：

- 区域级 Dirichlet handles
  - 不是只固定几个点，而是固定一片顶点集或一个夹持区域
- kinematic target trajectories
  - 用于“拉伸”“挤压”“夹住后拖动”
- 真正的 hyperelastic constitutive law
  - 至少需要 Neo-Hookean 级别，而不仅是 fixed corotated
- inversion-robust 材料实现
  - 否则强压缩场景很容易数值失稳
- 高质量 tet meshing pipeline
  - Stanford bunny 的表面网格不能直接拿来做体弹性，需要可靠四面体化
- 参考构型与体网格质量检查
  - 差的 tet quality 会直接毁掉牛顿收敛

每个时间步装配：

- deformation gradient $F$
- elastic energy
- elastic gradient
- elastic Hessian

### 3. Kinematic Obstacles

第一阶段建议统一都转成三角网格障碍物。

包括：

- ground plane
- sphere triangulation
- box triangulation
- 静态场景网格

先不要做解析球体/盒体接触特判。统一用 triangle mesh obstacle，系统更一致，后面 self-contact 和 body-body contact 也能共用流程。

### 4. Large-Scale Rigid Bodies

这是你最终目标里最容易低估的一块。

“支持大规模刚体碰撞”并不是当前这套 deformable IPC 内核自然顺带得到的能力，它需要额外的一条 rigid-specific 管线。至少要补齐：

- 刚体状态表示
  - 位移
  - 旋转
  - 线速度
  - 角速度
  - 惯性张量
- 刚体几何表示
  - sphere / box / capsule / convex hull
  - 复杂网格建议做 proxy，而不是直接拿高面数三角网格参与大规模碰撞
- 旋转相关 CCD
  - deformable 顶点线性插值式 CCD 不足以覆盖大角速度 rigid motion
- 接触 manifold / persistent contact
  - 大规模堆叠和 resting contact 需要更稳定的 manifold 管理
- 睡眠、岛分解、并行求解
  - 否则规模上去以后性能会很差
- broad phase 加速结构
  - 动态 AABB tree / BVH / spatial hash
- 摩擦堆叠稳定性
  - 这是大规模 rigid pile 最难看的数值点之一
- 场景级性能策略
  - sleeping
  - warm start
  - contact cache
  - island-level parallelism

如果你的“刚体碰撞”目标更偏游戏式大规模堆叠，那么它和 deformable IPC 的最优实现路径通常并不完全一致。现实上更可行的是：

- deformable 走 IPC 主干
- rigid 走独立的高性能刚体管线
- 两者通过 obstacle / proxy / coupling 做交互

只有当你明确要做 rigid IPC 或追求统一的非穿透优化框架时，才值得把 rigid 也纳入同一套 nonlinear optimizer。

## 推荐实现阶段

### Phase 0: 建底座

目标：把 IPC 的数学基础和工程骨架搭出来。

要做的事：

- 引入 `Eigen`
- 新建 `ipc/` 目录结构
- 实现统一状态向量
- 实现 DOF 映射
- 实现 `pbpt::math <-> Eigen` 转换
- 定义 `IPCBody / TetBody / ShellBody / ObstacleBody` 抽象
- 建立测试模板和数值检查工具

交付标准：

- 能构造一个最小 `IPCBody`
- 能构造最小 `TetBody` 与 `ShellBody`
- 能组装全局状态向量
- 能回写变形顶点到 renderer

### Phase 1: 无接触隐式求解

目标：先做“无碰撞”的优化型可变形求解器。

要做的事：

- inertial energy
- gravity energy
- tet elastic energy
- hyperelastic tet energy interface
- shell FEM energy interface
- Dirichlet constraints
- Newton solver
- sparse solve
- line search

推荐只做：

- 一个单 tetrahedron
- 一个小 tetrahedral mesh

这一阶段建议先把 `TetBody` 跑通，再给 `ShellBody` 只接入最小骨架，不要同时实现完整 cloth bending。

交付标准：

- 在无碰撞场景下稳定运行
- 大步长下比显式积分稳定
- 不出现 NaN
- 能完成 bunny 的“固定-拉伸-压缩”最小实验

### Phase 2: 接触几何与 Barrier

目标：先做 obstacle contact，不做 self-contact。

要做的事：

- point-triangle 距离、梯度、Hessian
- edge-edge 距离、梯度、Hessian
- broad phase
- 接触候选过滤
- barrier energy / gradient / Hessian

交付标准：

- tet vs floor 不穿透
- tet vs sphere obstacle 不穿透
- 接触梯度方向正确

### Phase 3: CCD 与 Collision-Free Line Search

目标：让每次 Newton 更新都保持无相交。

要做的事：

- point-triangle CCD
- edge-edge CCD
- conservative safe step 估计
- 将 CCD 接入 line search

交付标准：

- 即使 Newton 方向很激进，也不会穿透
- 高速撞击场景仍保持 collision-free

### Phase 4: Self-Contact

目标：支持 cloth/tet 的自碰撞。

要做的事：

- broad phase 支持 self pairs
- 邻接拓扑过滤
- self-contact barrier
- self-contact CCD

交付标准：

- cloth 折叠时不自穿
- tet 表面自接触时不穿透

### Phase 5: Friction

目标：支持稳定的接触摩擦。

要做的事：

- 切空间基
- lagged friction
- friction potential
- 摩擦参数暴露

交付标准：

- 斜面接触有合理停滞/滑动行为
- 不引入明显数值震荡

### Phase 6: Cloth Energy

目标：把 cloth 正式迁入统一 IPC 系统。

要做的事：

- codimensional membrane FEM energy
- bending energy
- cloth body 注册
- cloth scene sync

这一阶段的重点不是“再造一个 cloth world”，而是让 `ShellBody` 成为 `IPCSystem` 下的一等公民，与 `TetBody` 共用：

- barrier
- CCD
- broad phase candidate generation
- Newton framework
- line search
- global state 管理

交付标准：

- cloth drape 稳定
- cloth self-contact 稳定
- 与 obstacle 接触稳定

### Phase 7: 系统整合

目标：让 IPC 成为 RTR2 中可长期扩展的主干。

要做的事：

- `PhysicsSystem` 接管 `IPCSystem`
- editor 参数暴露
- reset / pause / debug draw
- 日志、统计、profiling

## 推荐首个里程碑

如果只做一个最有价值的里程碑，我建议是：

**tet FEM + kinematic floor/sphere obstacle + implicit Euler + barrier + CCD + friction 关闭**

原因：

- full volumetric deformation 更接近 IPC 论文主线
- 不需要先处理 cloth bending
- surface proxy 与 volumetric DOF 的关系更清楚
- 一旦这条路径打通，cloth 只是换掉弹性能

## 面向最终交付物的缺口分析

如果最终交付物是：

- hyperelastic Stanford bunny 被固定、拉伸、挤压
- cloth 等 codimensional FEM 模拟
- 大规模刚体碰撞

那么在当前这份 plan 基础上，还需要明确补齐下面这些能力。

### 1. Hyperelastic Bunny 所缺的能力

- 体网格生成流程
  - 需要从 Stanford bunny 表面网格生成高质量四面体网格
- hyperelastic constitutive law
  - fixed corotated 只能算起步
  - 最终展示“hyperelastic”应至少落到 Neo-Hookean 级别
- inversion-robust 材料与求解
  - 拉伸/挤压场景会逼近奇异形变
- 区域约束与夹具
  - 需要 handle region，而不是简单 pinned vertices
- 准静态或慢动态驱动模式
  - 如果你主要想展示拉伸/挤压形变，准静态求解模式会比纯动态更稳定也更可控
- 反力/能量观测
  - 如果要让 demo 有研究味道，最好能输出 force-displacement 或 energy curve

### 2. Codimensional Cloth 所缺的能力

- 明确的 shell FEM 实现
  - 不能停留在 mass-spring
- 自碰撞拓扑过滤
  - cloth self-contact 的非法邻接对过滤很关键
- 薄壳 bending 的稳定 Hessian
  - 这是 cloth 能否稳定的核心难点之一
- cloth 专用参数标定
  - 厚度
  - 面密度
  - 膜刚度
  - 弯曲刚度

### 3. 大规模刚体碰撞所缺的能力

- 一条独立的 rigid-body 高性能方案
  - 大规模 rigid pile 不适合直接复用当前 deformable-only IPC 主循环
- 旋转刚体的接触与 CCD
- manifold、warm start、sleeping、island solving
- 更激进的 broad phase 和并行化
- 简化几何代理
  - convex proxy / primitive proxy / compound shape

### 4. 工程化交付还缺的能力

- 场景 authoring
  - 夹持点、加载路径、障碍物、重置配置
- 录屏和 benchmark 场景
  - bunny stretch
  - bunny squeeze
  - cloth drop
  - cloth self-collision
  - rigid pile collapse
- profiling
  - broad phase
  - energy assembly
  - linear solve
  - CCD
- 参数管理
  - 每个 demo 一套稳定可复现实验参数

## 推荐的最终系统拆分

考虑你的最终目标，我建议最终不要把所有东西都硬塞进“一套 solver”：

- `IPCSystem`
  - `TetBody`
    - hyperelastic volumetric FEM
  - `ShellBody`
    - codimensional / shell FEM cloth
  - `ObstacleBody`
    - static / kinematic obstacle
  - obstacle/self-contact
- `RigidSystem`
  - 大规模刚体碰撞
  - 睡眠、岛分解、contact cache、并行化
- `CouplingLayer`
  - rigid <-> deformable 接触代理
  - kinematic handle / obstacle transfer

这比“纯统一求解器”更现实，也更容易做出最终成品。

## 不推荐的起步方式

以下路线都不建议作为第一步：

- 先在当前 `ClothWorld` 上打补丁加 barrier
- 先做 friction 再做 CCD
- 先做 self-contact 再做 obstacle contact
- 先重写 rigid body 再做 deformable IPC
- 先做高性能 broad phase，再做正确性

正确顺序应该始终是：

1. 无接触隐式动力学
2. 距离与导数
3. barrier
4. CCD
5. obstacle contact
6. self-contact
7. friction

## 测试建议

建议新增测试目录：

- `test/system/physics/ipc/geometry/`
- `test/system/physics/ipc/energy/`
- `test/system/physics/ipc/contact/`
- `test/system/physics/ipc/ccd/`
- `test/system/physics/ipc/solver/`

最关键的测试：

- 距离函数与有限差分梯度对比
- Hessian 与有限差分 Jacobian 对比
- 单 tetrahedron 弹性能导数检查
- barrier 导数检查
- CCD 安全步长检查
- pinned DOF 消元正确性
- 牛顿迭代在极端 case 下不出 NaN

### 强烈建议做的数值检查

每实现一个能量项，都做：

- energy finite difference gradient check
- gradient finite difference Hessian check

这是从零实现 IPC 时最重要的工程保障之一。

## 调试建议

建议尽早做这些调试输出：

- 当前能量分项
- 当前最小接触距离
- 当前 barrier 项数量
- 当前 Newton 迭代次数
- 当前 line search 步长
- 当前最大梯度范数

建议尽早做这些可视化：

- 接触点/接触对高亮
- 自碰撞候选线段高亮
- pinned vertices 高亮
- 线搜索失败时的 candidate geometry 可视化

## 最终推荐的代码落地形态

- `PhysicsSystem`
  - `RigidBodyWorld`
  - `IPCSystem`
- `IPCSystem`
  - global state
  - body registry
  - collision mesh builder
  - candidate generation
  - active constraints
  - energy assembler
  - newton solver
  - ccd line search
  - scene sync

这种结构最符合“从零实现 IPC 主干”的目标，因为它把系统的共享复杂度放在 solver 和 contact pipeline，而不是继续按对象类别拆碎。

## 结论

对 RTR2 来说，从零实现 IPC 的正确路线不是“把现有 cloth 升级一下”，而是：

- 新建统一的 `IPCSystem`
- 用 `Eigen` 统一求解器内部数值类型
- 先做 tet FEM 的无接触隐式求解
- 再补 barrier
- 再补 CCD
- 再补 self-contact 和 friction
- 最后再把 cloth 和更复杂耦合迁进来

如果你按这个顺序推进，难点会被拆成一组可验证的小模块；如果反过来从 cloth self-contact 或 rigid coupling 开始，项目复杂度会陡增。
