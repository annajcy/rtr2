# 物理系统总览 (Overview)

本文档描述当前 `rtr::system::physics` 的真实运行时结构。现在仓库中的 physics 目录已经收拢为两个方向：

- **已接入运行时的刚体子系统**：`rigid_body/`
- **正在建设中的 IPC/FEM 子系统**：`ipc/`

当前 `PhysicsSystem` 只持有 `RigidBodyWorld`。`ipc/` 目录已经开始放核心数据结构，但还没有接到 runtime 主循环里。

## 当前能力与边界

当前 physics system 已经包含：

- 刚体状态积分、基础碰撞检测与碰撞响应求解。
- scene graph 与 rigid-body world 的双向同步。
- IPC/FEM 的基础数据结构雏形：`IPCState`、`IPCBodyInfo`、`TetBody`、`ObstacleBody`、tet/mesh 转换。

当前明确**不包含**：

- cloth runtime。
- implicit IPC solver 的主循环接入。
- IPC contact / barrier / CCD。
- tet 弹性能、Newton、line search。
- rigid-body / IPC coupling。

## 架构总览

1. **`PhysicsSystem`**
   当前物理运行时的总容器，只持有一个 `RigidBodyWorld`。它本身只负责 world 内部求解，不直接做 scene 同步。

2. **Framework Integration (`src/rtr/framework/integration/physics/`)**
   负责把 scene graph 中的组件状态同步到物理运行时，再把模拟结果写回 scene graph。当前权威入口是 `step_scene_physics(scene, physics_system, dt)`。

3. **`RigidBodyWorld`**
   保存刚体、附着碰撞体、接触快照和冲量求解过程。当前算法是 semi-implicit Euler 积分 + 接触生成 + PGS 速度求解 + penetration correction。

4. **`ipc/`**
   这是未来的 FEM/IPC 子系统目录。当前阶段只放“连续场离散化后需要的数据结构”，还不负责真正的 runtime 推进。

## Fixed Tick 运行数据流

当前 fixed tick 路径如下：

```text
AppRuntime fixed tick
    |
    v
step_scene_physics(scene, physics_system, dt)
    |
    +--> sync_scene_to_rigid_body(scene, rigid_body_world)
    +--> PhysicsSystem::step(dt)
    |       |
    |       \--> RigidBodyWorld::step(dt)
    \--> sync_rigid_body_to_scene(scene, rigid_body_world)
```

这条路径有两个关键点：

- `PhysicsSystem::step()` 只做 world 内部求解，不负责 scene/physics 边界。
- `ipc/` 目前还不在这条 fixed tick 数据流里。

## 状态权威与所有权

| 位置 | 保存内容 | 谁是权威 |
| --- | --- | --- |
| Scene Graph | `GameObject`、节点层级、组件挂载、渲染可见对象 | 框架层 |
| `RigidBodyWorld` | 刚体位置/速度/朝向/角速度、collider、solver contacts | Dynamic rigid body 运行时权威 |
| `ipc::IPCState` | 未来 IPC/FEM 的全局节点坐标、速度、质量对角 | 未来 deformable runtime 权威 |

因此：

- Dynamic rigid body 开始模拟后，不应再把 scene graph transform 当成权威状态。
- `IPCState` 的设计目标是把所有 deformable body 的节点自由度统一堆叠到一个全局向量里，后续能量、梯度、Hessian 都围绕这个向量做。

## IPC 数据结构：`IPCState` 是什么

`ipc::IPCState` 是未来 IPC/FEM 求解器的**全局状态容器**。它的角色不是“某一个 tet body 的局部缓存”，而是把所有 deformable body 的节点自由度统一堆叠成一个有限维向量。

这背后的理论出发点是：

- 连续系统原本描述的是位移场 `x(X, t)`，定义在整个参考域上。
- 做线性有限元离散后，连续场不再直接存储，而是由有限个节点坐标插值表示。
- 对 3D tet 网格来说，1 个顶点对应 3 个自由度，所以 `N` 个顶点最终变成 `3N` 个未知量。

于是 `IPCState` 就承载这组全局未知量。

如果把参考构型中的材料点记为 `X \in \Omega^0`，线性有限元的近似写法是：

$$
\hat{x}(X) = \sum_{a=1}^{N} x_a N_a(X)
$$

这里：

- `N_a(X)` 是节点 `a` 的 shape function
- `x_a \in \mathbb{R}^3` 是节点 `a` 的当前坐标

也就是说，连续位移场不再直接作为函数存储，而是被节点变量 `{x_a}` 代表。把所有节点坐标堆起来，就得到全局状态向量：

$$
\mathbf{x} =
\begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_N
\end{bmatrix}
\in \mathbb{R}^{3N}
$$

这正是 `IPCState::x` 的数学含义。

### 字段含义

- `x`
  当前位形，也就是当前时刻所有节点坐标拼成的全局向量。长度是 `3N`。

  $$\mathbf{x} \in \mathbb{R}^{3N}$$

- `x_prev`
  上一步位形。后续做隐式时间积分时，它会参与构造惯性能中的 `x_hat` 或速度更新。

  $$\mathbf{x}_{prev} \in \mathbb{R}^{3N}$$

- `v`
  所有节点速度拼成的全局向量。长度同样是 `3N`。

  $$\mathbf{v} \in \mathbb{R}^{3N}$$

- `mass_diag`
  对角质量向量，而不是完整质量矩阵。当前设计采用 lumped mass，也就是把每个顶点质量平均分配到对应的三个平移自由度上，因此长度也是 `3N`。

  如果顶点 `i` 的标量质量是 `m_i`，那么在对角形式里对应的是：

  $$
  \mathbf{M} = \mathrm{diag}(m_1, m_1, m_1, m_2, m_2, m_2, \ldots, m_N, m_N, m_N)
  $$

  `mass_diag` 存的就是这个对角矩阵的对角线。

### 为什么是一个全局向量

求解阶段不会把每个 body 分开各自做一个小系统，而是把所有参与求解的 body 统一装配成一个大系统。这样后续：

- 惯性能量
- 重力势能
- tet 弹性能
- 接触 barrier
- Newton 梯度/Hessian

都可以写成对同一个全局变量 `x` 的函数。

这也是 `IPCBodyInfo::dof_offset` 存在的原因：它负责说明“某个 body 的局部顶点”在 `IPCState` 这个全局向量里从哪里开始。

### DOF 布局

当前布局是最直接的 vertex-major：

```text
x = [x0x, x0y, x0z, x1x, x1y, x1z, ..., x(N-1)z]^T
```

写成数学形式就是：

$$
\mathbf{x} =
[x_{0x}, x_{0y}, x_{0z}, x_{1x}, x_{1y}, x_{1z}, \ldots, x_{(N-1)z}]^T
$$

所以：

- 顶点 `i` 的位置起始下标是 `3 * i`
- 顶点 `i` 的速度起始下标也是 `3 * i`
- 顶点 `i` 的质量对角也是 `mass_diag[3*i + 0 .. 3*i + 2]`

对应代码接口就是：

- `vertex_count() = x.size() / 3`
- `dof_count() = x.size()`
- `position(i)` 返回 `x.segment<3>(3 * i)`

例如当 `N = 4` 时：

- `dof_count() = 12`
- 顶点 2 对应 `x[6], x[7], x[8]`

也就是：

$$
x_2 =
\begin{bmatrix}
\mathbf{x}_6 \\
\mathbf{x}_7 \\
\mathbf{x}_8
\end{bmatrix}
$$

### 为什么 `mass_diag` 也是 `3N`

从物理上说，一个顶点通常只有一个标量质量 `m_i`。但在求解器里更方便的形式是把它扩成三个平移自由度共享的对角项：

```text
[m_i, m_i, m_i]
```

这样后续惯性能、梯度和 Hessian 都可以直接按 DOF 做逐元素运算，而不需要再单独构造一个稀疏质量矩阵。

例如隐式积分里常见的惯性能写法是：

$$
E_I(\mathbf{x}) = \frac{1}{2h^2}(\mathbf{x} - \hat{\mathbf{x}})^T \mathbf{M} (\mathbf{x} - \hat{\mathbf{x}})
$$

当 `\mathbf{M}` 取对角形式后，这个式子就能直接写成逐元素求和：

$$
E_I(\mathbf{x}) = \frac{1}{2h^2} \sum_{j=1}^{3N} \mathrm{mass\_diag}_j (x_j - \hat{x}_j)^2
$$

这就是 `mass_diag` 用 `VectorXd` 而不是稀疏矩阵存储的直接原因。

### 当前实现上的取舍

`IPCState` 本身保持非常薄：

- 不持有拓扑
- 不知道 tet connectivity
- 不知道材料参数
- 不做能量计算

它只负责一件事：**以求解器友好的方式存储全局自由度**。

拓扑和材料属于 model 层对象；把状态、几何、材料分开，可以避免“全局自由度”和“局部几何/材料元数据”耦在一起。

## IPC 数据结构：为什么需要 `IPCBodyInfo`

`ipc::IPCBodyInfo` 是一个很小的元数据结构，但它承担的是“单个 body 如何映射到全局求解向量”的角色。

字段含义如下：

- `type`
  表示 body 的种类。当前预留了 `Tet`、`Shell`、`Obstacle` 三类，用来支撑后续统一管理和按类型分派。

- `dof_offset`
  表示这个 body 在全局 `IPCState` 向量中的起始 DOF 下标。因为 1 个顶点对应 3 个 DOF，所以它指向的是 `x` / `v` / `mass_diag` 里的某个 `3 * vertex_offset` 位置。

- `vertex_count`
  表示这个 body 自己拥有多少个顶点。它和 `dof_offset` 一起决定了该 body 在全局向量里覆盖的连续区间。

- `enabled`
  表示装配和求解阶段是否暂时跳过这个 body。这个开关的价值在于“先保留 body，再决定是否参与当前求解”，而不是频繁增删容器元素。

一个直观例子：

- body A 有 4 个顶点，对应 12 个 DOF
- body B 有 10 个顶点，对应 30 个 DOF

那全局 `IPCState::x` 的长度就是 `42`。此时：

- A 的 `dof_offset = 0`，`vertex_count = 4`
- B 的 `dof_offset = 12`，`vertex_count = 10`

于是：

- A 使用 `x.segment(0, 12)`
- B 使用 `x.segment(12, 30)`

这就是为什么 `IPCBodyInfo` 看起来字段很少，但它实际上定义了“body 局部顶点编号”到“全局优化变量编号”的映射边界。

## IPC 数据结构：`TetGeometry` 和 `TetBody` 为什么拆开

现在 tet 相关的数据结构被明确拆成两层：

- `ipc::TetGeometry`
  只负责四面体网格的几何与参考构型预计算。

- `ipc::TetBody`
  在 `TetGeometry` 之上再补 body 元数据、材料参数、边界条件和顶点质量。

这样拆的原因是：tet mesh 的**几何问题**和 tet body 的**物理问题**不是一回事。

- 几何层关心：顶点、tet 拓扑、体积、`D_m^{-1}`、surface proxy、未来的 tet/triangle 转换
- 物理层关心：密度、材料参数、固定点、质量装配、后续进入全局系统的方式

从数据上看：

- `TetGeometry` 持有：
  - `rest_positions`
  - `tets`
  - `Dm_inv`
  - `rest_volumes`

- `TetBody` 持有：
  - `info`
  - `geometry`
  - `vertex_masses`
  - `density`
  - `youngs_modulus`
  - `poisson_ratio`
  - `fixed_vertices`

其中真正决定 FEM 离散的是 `TetGeometry`：

- `rest_positions` 给出所有节点在参考构型中的位置
- `tets` 用顶点索引说明“哪些 4 个点组成一个四面体单元”

所以一个大的 3D 体对象，并不是直接存成一个连续函数，而是被拆成很多共享顶点的小四面体：

$$
\Omega^0 \approx \bigcup_{e=1}^{n_{tet}} T_e
$$

每个 `T_e` 是一个线性四面体单元。

### `D_m` 和 `D_m^{-1}`：为什么属于 `TetGeometry`

对一个 tet 单元

$$
e = (v_0, v_1, v_2, v_3)
$$

记参考构型中的四个顶点为

$$
X_0, X_1, X_2, X_3
$$

当前构型中的四个顶点为

$$
x_0, x_1, x_2, x_3
$$

FEM 的核心局部量是 deformation gradient：

$$
F = \frac{\partial x}{\partial X}
$$

在线性四面体里，单元内部的位移插值是 affine，因此 `F` 在整个单元内是常量。它可以直接由“参考边向量”和“当前边向量”确定：

$$
F(X_1 - X_0) = x_1 - x_0
$$

$$
F(X_2 - X_0) = x_2 - x_0
$$

$$
F(X_3 - X_0) = x_3 - x_0
$$

把三条关系并成矩阵，得到

$$
F
\underbrace{
\begin{bmatrix}
X_1 - X_0 & X_2 - X_0 & X_3 - X_0
\end{bmatrix}
}_{D_m}
=
\underbrace{
\begin{bmatrix}
x_1 - x_0 & x_2 - x_0 & x_3 - x_0
\end{bmatrix}
}_{D_s}
$$

因此：

$$
\boxed{F = D_s D_m^{-1}}
$$

这就是 `TetGeometry::precompute_rest_data()` 要缓存 `Dm_inv` 的根本原因：

- `D_s` 依赖当前位形，每步都会变
- `D_m` 只依赖参考构型，整个仿真过程中不变

所以把 `D_m^{-1}` 预先算好以后，后续每步只需要组 `D_s` 再做一次矩阵乘法，就能得到单元的 deformation gradient。

代码中的定义正对应这个公式：

```cpp
Dm.col(0) = X1 - X0;
Dm.col(1) = X2 - X0;
Dm.col(2) = X3 - X0;
```

### Rest volume：为什么是 `|det(D_m)| / 6`

`D_m` 的三列是从 `X_0` 出发到另外三个顶点的边向量。线性代数里，三列向量张成的平行六面体体积是：

$$
|\det(D_m)|
$$

而一个四面体的体积恰好是对应平行六面体的六分之一，所以：

$$
\boxed{V_e = \frac{|\det(D_m)|}{6}}
$$

这就是 `TetGeometry::rest_volumes[i]` 的来源。

这里还有一个重要细节：`det(D_m)` 的符号带有单元取向信息。

- `det(D_m) > 0`：取向正常
- `det(D_m) = 0`：四点共面，单元退化
- `det(D_m) < 0`：顶点顺序翻转

从当前工程目标来看，退化 tet 是必须拦掉的，因为后续：

- `D_m^{-1}` 不存在或数值极差
- 单元体积无意义
- deformation gradient 和弹性能都会坏掉

所以 `TetGeometry::precompute_rest_data()` 里对 `det(D_m)` 做退化检查是必要的。

### `J = det(F)` 和体积变化

体积比也可以直接从 `F` 的行列式得到：

$$
J = \det(F)
$$

代入

$$
F = D_s D_m^{-1}
$$

可得：

$$
J = \frac{\det(D_s)}{\det(D_m)}
$$

又因为当前体积和参考体积都各自等于对应行列式除以 6，所以：

$$
J = \frac{V_{current}}{V_{rest}}
$$

这就是为什么 FEM 和超弹性能里常常把 `J` 解释为局部体积变化率。

### 质量预计算：为什么是每个 tet 平均分给 4 个顶点

从 weak form 离散后，严格的 consistent mass matrix 是：

$$
M_{ab} = \int_{\Omega^0} \rho_0 N_a N_b \, dX
$$

这里：

- `\rho_0` 是参考密度
- `N_a, N_b` 是节点 shape function

这个矩阵一般是稀疏但非对角的。图形学仿真里更常用的是 **mass lumping**，即把质量集中到对角线上，近似成对角质量矩阵。

对于线性四面体，一个单元的总质量是：

$$
m_e = \rho V_e
$$

最简单稳定的 lumping 方法，就是把它平均分给四个顶点：

$$
\boxed{m_a \mathrel{+}= \frac{\rho V_e}{4}}
\quad \text{for each vertex } a \in e
$$

于是每个顶点最终质量是它所属所有单元贡献之和：

$$
m_i = \sum_{e \ni i} \frac{\rho V_e}{4}
$$

代码里的

```cpp
vertex_mass[v] += density * rest_volume / 4.0;
```

正对应这个公式。

### 为什么 `vertex_masses` 仍然放在 `TetBody`

从严格分层来说，`IPCState::mass_diag` 才是最终用于求解器的全局质量表示；而 `TetGeometry` 只负责几何，不负责物理质量。所以在 `TetBody` 里先把每顶点质量算成 `vertex_masses` 更合理，因为：

- 质量来自 `geometry.rest_volumes` 和 body 自己的 `density`
- 还没有进入全局装配阶段
- 后续 `IPCSystem` 可以再把它展开成

$$
\mathrm{mass\_diag} =
[m_0, m_0, m_0, m_1, m_1, m_1, \ldots]^T
$$

这样 model 层负责“单体几何和局部物理量”，system 层负责“全局状态拼接”，职责更清楚。

### `fixed_vertices`：为什么也留在 `TetBody`

`fixed_vertices` 是一个顶点级布尔 mask。若顶点 `i` 被固定，则：

$$
x_i = x_i^{fixed}
$$

它的意义不是“立刻在 `TetBody` 里修改位置”，而是给后续求解器或系统装配一个最简单的 Dirichlet 元数据来源：

- 哪些顶点是 pinned
- 哪些 DOF 后续需要消元或冻结

当前先用 `std::vector<bool>` 是合理的最小实现；更复杂的 region handle、kinematic target、局部约束组，等 solver 接起来后再扩展。

### `TetBody::precompute()` 的本质

现在 `precompute()` 的职责是分层的：

- `TetGeometry::precompute_rest_data()` 固化只依赖参考构型的几何量
- `TetBody::precompute()` 在此基础上补齐 fixed mask 和 `vertex_masses`

综合起来，整个流程做的不是“仿真”，而是把原始 mesh 输入变成可用于 FEM/IPC 装配的数据：

- `D_m^{-1}`：后续用于 `F = D_s D_m^{-1}`
- `V_e`：后续用于能量积分和质量计算
- `vertex_masses`：后续用于全局对角质量装配

这一步完成后：

- `TetGeometry` 是“可复用的 tet 几何表示”
- `TetBody` 是“带物理属性的 tet body”

## Tet Block 生成工具：`generate_tet_geometry_block()` 的算法

除了从外部 mesh 读取数据，当前还需要一个**程序化生成 tet mesh** 的工具，方便：

- 单元测试
- smoke test
- 最小 demo

当前实现是 [`generate_tet_geometry_block()`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/model/tet_body.hpp#L129)，以及它的便捷包装 [`generate_tet_block()`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/model/tet_body.hpp#L204)。

### 输入参数的几何含义

函数输入是：

- `nx, ny, nz`
  block 在 x/y/z 三个方向上的 cube 个数
- `spacing`
  相邻规则网格点之间的间距
- `origin`
  block 左下后角的起始位置

它先构造的是一个规则笛卡尔网格，因此顶点数为：

$$
(n_x + 1)(n_y + 1)(n_z + 1)
$$

因为 `n_x` 个 cube 需要 `n_x + 1` 层网格点。

### Step 1: 先生成规则网格顶点

算法先遍历：

$$
0 \le i \le n_x,\quad 0 \le j \le n_y,\quad 0 \le k \le n_z
$$

把每个格点写成：

$$
X_{ijk} = \text{origin} + \text{spacing} \cdot (i, j, k)^T
$$

代码里还定义了一个一维索引映射：

$$
\text{vertex\_index}(i,j,k) = i + (n_x + 1)\bigl(j + (n_y + 1)k\bigr)
$$

这样三维格点 `(i,j,k)` 就能映射成 `rest_positions` 里的线性下标。

### Step 2: 每个 cube 固定切成 6 个 tetrahedron

对每个 cube，先取它的 8 个角点：

```text
v000, v100, v010, v110,
v001, v101, v011, v111
```

然后用固定的 **6-tet-per-cube** 切法，把一个 cube 分成：

$$
(v000, v100, v110, v111)
$$

$$
(v000, v100, v101, v111)
$$

$$
(v000, v001, v101, v111)
$$

$$
(v000, v001, v011, v111)
$$

$$
(v000, v010, v110, v111)
$$

$$
(v000, v010, v011, v111)
$$

这相当于让所有 tet 都围绕 cube 的体对角线：

$$
v000 \leftrightarrow v111
$$

展开。这样实现简单、规则，特别适合测试。

因此总 tetrahedron 数量是：

$$
6 n_x n_y n_z
$$

### Step 3: 为什么还要做 orientation 修正

即使几何点集是对的，tet 的局部顶点顺序不同，也会让：

$$
\det(D_m)
$$

的符号不同。

对一个 tet：

$$
D_m =
\begin{bmatrix}
X_1 - X_0 & X_2 - X_0 & X_3 - X_0
\end{bmatrix}
$$

如果：

$$
\det(D_m) < 0
$$

说明这个 tet 的局部取向是反的。后续 FEM 预计算更希望所有单元都遵守统一的正体积约定，所以当前实现会：

- 先算一次 `det(D_m)`
- 若为负，交换两个局部顶点顺序

这一步不是改几何，而是在统一 tet 的局部编号顺序。

### Step 4: 为什么这个工具适合当前阶段

`generate_tet_geometry_block()` 的目标不是通用 tetrahedralizer，而是生成一个：

- 规则
- 健康
- 可预测
- 无外部依赖

的体网格输入。

所以它特别适合：

- `TetGeometry::precompute_rest_data()` 测试
- `TetBody::precompute()` 测试
- 最小 deformable block demo

### 小结

这个 block 生成算法本质上就是：

1. 先生成规则三维 lattice 顶点
2. 把每个 cube 固定切成 6 个 tetrahedron
3. 对每个 tet 做 orientation 修正，保证后续是正体积约定

## 当前 Tet/Mesh Conversion 的边界

当前 [`tet_mesh_convert.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp) 的职责被刻意限制在一个轻量 conversion 层里：

- 已支持：`TetGeometry` / `TetBody` / `IPCState::x` 到 surface `ObjMeshData`
- 已支持：`ObjMeshData` 到 position/triangle helper 数组
- 未支持：`ObjMeshData` 直接生成 volumetric `TetGeometry`

也就是说：

- `tet_to_mesh(...)` 和 `update_mesh_positions(...)` 是正式的渲染写回路径
- `mesh_positions_to_eigen(...)` 和 `mesh_triangles(...)` 只是为未来外部 tetrahedralizer 预留的输入 helper

未来目标链路是：

```text
ObjMeshData
  -> mesh_positions_to_eigen()
  -> mesh_triangles()
  -> external tetrahedralizer (future)
  -> TetGeometry
```

其中真正的 volumetric meshing 明确不在当前实现范围内。

## 当前 `tet -> mesh` 算法实现

当前 `tet -> mesh` 的实现目标很具体：把体网格 `TetGeometry` 或当前求解状态 `IPCState::x`，稳定转换成渲染侧的 `ObjMeshData`。

它不是重新 meshing，而是一个 **surface extraction + vertex remapping + normal recomputation** 的过程。实现文件是 [`tet_mesh_convert.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp)。

### Step 1: 先从 tet mesh 中找出外表面

一个四面体有 4 个三角面。若某个三角面被两个 tetrahedron 同时共享，它就在体内部；若它只属于一个 tetrahedron，它就在外表面。

所以算法的核心判据是：

$$
\text{surface face} \iff \text{face appears exactly once}
$$

代码里会遍历每个 tet：

$$
(v_0, v_1, v_2, v_3)
$$

并枚举它的 4 个面：

$$
(v_0, v_1, v_3),\ (v_1, v_2, v_3),\ (v_2, v_0, v_3),\ (v_0, v_2, v_1)
$$

这里做了两件事：

- 用排好序的三元组当 key，识别“这是不是同一个几何面”
- 额外保存原始有向顶点顺序，方便后面直接输出三角形 winding

因此 `extract_tet_surface(...)` 的结果是：

- `surface_indices`
  表面三角形的顶点编号，但仍然引用 **原始 tet 顶点 id**
- `surface_vertex_ids`
  出现在表面上的唯一顶点 id 集合

这一步本质上是把：

$$
\Omega^0 \approx \bigcup_e T_e
$$

转换成边界离散：

$$
\partial \Omega^0 \approx \bigcup_f \triangle_f
$$

### Step 2: 把体网格顶点号压缩成 surface mesh 顶点号

提取出外表面以后，还不能直接得到 `ObjMeshData`，因为 `surface_indices` 里引用的是原始 tet 顶点号，而渲染 mesh 只应该存真正出现在表面的顶点。

所以第二步要做一个重映射：

$$
\text{old tet vertex id} \rightarrow \text{new surface vertex id}
$$

例如原始体网格顶点可能是：

```text
0, 1, 2, 3, 4, 5, 6, ...
```

但表面只用了：

```text
[0, 2, 5, 7]
```

那新的 surface mesh 顶点编号就压成：

```text
0 -> 0
2 -> 1
5 -> 2
7 -> 3
```

于是：

- `ObjMeshData.vertices` 只存 4 个顶点
- `ObjMeshData.indices` 改写成新的紧凑 0-based index

这一步的价值是：

- 不把内部顶点带进渲染 mesh
- 渲染顶点缓冲更小
- 后续每帧更新时可以只更新真正可见的表面顶点

### Step 3: 顶点位置从哪里来

现在 `tet_to_mesh(...)` 有两个入口，对应两种位置来源：

1. `tet_to_mesh(const Eigen::VectorXd& positions, ...)`

   这里 `positions` 就是当前时刻的全局 DOF 向量，布局是：

   $$
   \mathbf{x} = [x_{0x}, x_{0y}, x_{0z}, x_{1x}, x_{1y}, x_{1z}, \ldots]^T
   $$

   因此顶点 `i` 的空间位置来自：

   $$
   x_i =
   \begin{bmatrix}
   \mathbf{x}_{3i} \\
   \mathbf{x}_{3i+1} \\
   \mathbf{x}_{3i+2}
   \end{bmatrix}
   $$

   这是当前真正的写回路径：`IPCState::x -> ObjMeshData`

2. `tet_to_mesh(const TetGeometry& geometry, ...)`

   这里位置直接来自 `geometry.rest_positions`。这个入口主要用于：

   - 生成 rest shape 的可视化 mesh
   - 测试 surface extraction 和 remap 是否正确

### Step 4: 为什么还要重算法线

渲染侧 `ObjMeshData` 需要法线，但 tet solver 输出的是顶点位置，不会直接给 surface normal。

所以在构建完三角 mesh 后，算法会按三角面重新累积顶点法线。对一个三角形 `(i_0, i_1, i_2)`，面法线按：

$$
n_f = \mathrm{normalize}\left((p_1 - p_0) \times (p_2 - p_0)\right)
$$

再把它加到三个顶点上，最后逐顶点归一化：

$$
n_i = \mathrm{normalize}\left(\sum_{f \ni i} n_f\right)
$$

这和 [`obj_io.hpp`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/src/rtr/utils/obj_io.hpp) 在“OBJ 无输入法线时”的策略是一致的。

### Step 5: 为什么 `update_mesh_positions()` 可以更轻

`extract_tet_surface(...)` 和顶点重映射只依赖 tet 拓扑，不依赖当前位形。因此只要 tet connectivity 不变，这个映射就可以缓存。

于是每一帧更新时，不需要重新：

- 枚举所有 tet 面
- 重新找 surface face
- 重新建立 old->new 顶点映射

只需要：

1. 用缓存好的 `surface_vertex_ids` 读取新的表面顶点位置
2. 覆写 `mesh.vertices[i].position`
3. 重算法线

这就是 `update_mesh_positions(...)` 的意义：它假设 surface topology 已经固定，因此把每帧开销压缩成：

$$
\text{surface vertex position update} + \text{normal recomputation}
$$

而不是每帧完整重建整个 surface extraction 过程。

### 小结

当前的 `tet -> mesh` 算法可以概括成：

1. 从每个 tet 的 4 个面中找出“只出现 1 次”的 boundary face
2. 收集边界顶点并建立 old tet id 到 new surface id 的映射
3. 从 `IPCState::x` 或 `TetGeometry::rest_positions` 读取顶点位置
4. 构造紧凑的 `ObjMeshData`
5. 重新计算顶点法线

所以它解决的是：

- **体网格怎么变成可渲染表面 mesh**
- **当前求解 DOF 怎么写回 renderer**

而不是：

- **triangle surface mesh 怎么反推 volume tet mesh**

## 算法地图

| 子系统 | 主要状态 | 当前算法/职责 | 主要入口 | 当前不做 |
| --- | --- | --- | --- | --- |
| Rigid Body | 刚体平移/旋转状态、碰撞体、solver contacts | semi-implicit Euler + contact generation + PGS + positional correction | `RigidBodyWorld::step()` | CCD、persistent manifold、warm starting |
| IPC Data Layer | 全局 DOF、tet rest state、tet->mesh 写回、mesh helper conversion、body 元数据 | 连续场离散后的状态承载与预计算 | `ipc/core/`、`ipc/model/` | runtime step、能量装配、求解器 |
| Framework Integration | scene graph、physics world | fixed tick scene/physics sync | `step_scene_physics()` | IPC scene sync |

## 目录与代码入口

当前最关键的入口文件是：

- `src/rtr/system/physics/physics_system.hpp`
- `src/rtr/framework/integration/physics/scene_physics_step.hpp`
- `src/rtr/framework/integration/physics/rigid_body_scene_sync.hpp`
- `src/rtr/system/physics/rigid_body/rigid_body_world.hpp`
- `src/rtr/system/physics/ipc/core/ipc_state.hpp`
- `src/rtr/system/physics/ipc/model/ipc_body.hpp`
- `src/rtr/system/physics/ipc/model/tet_body.hpp`

## 延伸阅读

- 运行时集成：`runtime-integration.zh.md`
- 刚体算法与碰撞响应：`rigid-body-dynamics.zh.md`
