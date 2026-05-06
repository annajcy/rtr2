# RigidBodySystem 理论与实现

本文档说明当前 RTR2 里已经实现的刚体子系统。它讨论的是当前 `PhysicsSystem -> rb::RigidBodySystem -> collision/ + solver` 这一条真实路径，而不是历史上更宽泛的 physics runtime 设想，也不是整个 physics runtime 的总览。

## 运行时位置

在当前引擎里，刚体只是 physics system 的一部分：

- scene/physics 边界由 `step_scene_physics(...)` 处理；
- `step_scene_physics(...)` 会显式调用 `rb::RigidBodySystem::step(dt)`；
- 本页只聚焦 `rb::RigidBodySystem` 内部使用的数值算法与接触求解流程。

## 状态变量

当前刚体运行时围绕这几类状态展开：

| 状态 | 作用 |
| --- | --- |
| `translation.position` | 刚体质心位置 |
| `translation.linear_velocity` | 线速度 |
| `rotation.orientation` | 四元数朝向 |
| `rotation.angular_velocity` | 角速度 |
| `forces.accumulated_force` | 当前帧累计外力 |
| `forces.accumulated_torque` | 当前帧累计力矩 |
| `mass / inverse_mass` | 平移质量属性 |
| `inverse_inertia_tensor_ref` | 刚体局部参考系下的逆惯性张量 |

Dynamic 刚体的权威运行时状态保存在 `rb::RigidBodySystem`，而 scene graph 中的 transform 只是输入源或回写目标。

## 当前每帧算法总流程

`rb::RigidBodySystem::step(dt)` 的核心流程如下：

```text
for each rigid body:
    integrate_body(body, dt)

contacts = collect_contacts()
solver_contacts = build_solver_contacts(contacts)

for velocity iteration in [0, velocity_iterations):
    for each solver contact:
        apply_velocity_impulses(contact)

for position iteration in [0, position_iterations):
    for each solver contact:
        apply_position_correction(contact)

for each rigid body:
    clear accumulated force / torque
```

这是一条典型的“先积分、再构建接触快照、再做冲量求解、最后清力”的离散刚体管线。

### 代码对应关系

- `rb::RigidBodySystem::step(...)`
- 文件：`src/rtr/system/physics/rigid_body/rigid_body_system.hpp`

## 平移积分

当前线性积分使用 semi-implicit Euler：

$$
\mathbf{a} = m^{-1} \mathbf{f}
$$

$$
\mathbf{v}^{k+1} = \mathbf{v}^k + \mathbf{a} \Delta t
$$

$$
\mathbf{v}^{k+1} \leftarrow \mathbf{v}^{k+1} \cdot \text{linear\_decay}
$$

$$
\mathbf{x}^{k+1} = \mathbf{x}^k + \mathbf{v}^{k+1} \Delta t
$$

其中：

- 若 `use_gravity()` 为 true，会先把 `gravity() * mass` 加到力累加器；
- `linear_decay` 是一个显式的速度衰减系数；
- 更新顺序是先速度、后位置，因此属于 semi-implicit Euler，而不是最朴素的 explicit Euler。

### 代码对应关系

- `rb::RigidBodySystem::integrate_body(...)`
- `rb::RigidBodySystem::gravity()`

## 旋转积分

旋转部分同样是“先角速度、后朝向”的更新：

1. 把局部参考系中的逆惯性张量转到世界空间：

$$
I^{-1}_{world} = R I^{-1}_{ref} R^T
$$

2. 用累计力矩计算角加速度：

$$
\boldsymbol{\alpha} = I^{-1}_{world} \boldsymbol{\tau}
$$

3. 更新角速度并施加 `angular_decay`：

$$
\boldsymbol{\omega}^{k+1} = \boldsymbol{\omega}^{k} + \boldsymbol{\alpha} \Delta t
$$

4. 用四元数导数推进朝向，并在每步后归一化：

$$
\dot{q} = \frac{1}{2} \omega_q q
$$

其中 $\omega_q = [0, \omega_x, \omega_y, \omega_z]$。

当前实现选择四元数，是因为它比欧拉角更稳定，也比直接操作旋转矩阵更紧凑。

### 代码对应关系

- `rb::RigidBodySystem::integrate_body(...)`
- `rb::RigidBodySystem::inverse_inertia_tensor_world(...)`

## 接触生成：从碰撞检测到 solver contact

当前系统把“几何接触生成”和“刚体 solver 数据”明确分成两层。

### 第一步：枚举碰撞体对

`collect_contacts()` 会遍历所有 collider pair，并跳过：

- 同一个 rigid body 上的 collider 对；
- 两边都不是 awake dynamic body 的组合。

通过这一步，系统避免对 static-static、static-kinematic 这类不会产生当前 solver 结果的组合做无效求解。

### 第二步：调用 `collision/` 生成几何接触

`generate_contact_from_pair(...)` 会把 collider 先转成 `WorldCollider`，然后调用：

- `rb::ContactPairTrait<rb::SphereShape, rb::SphereShape>::generate(...)`
- `rb::ContactPairTrait<rb::SphereShape, rb::BoxShape>::generate(...)`
- `rb::ContactPairTrait<rb::BoxShape, rb::PlaneShape>::generate(...)`
- 等等

`collision/` 只返回几何层的 `ContactResult`，里面包含穿透深度、接触点、法线等信息。

### 第三步：提升成 `SolverContact`

`build_solver_contacts(...)` 会把几何接触提升成 solver 需要的结构，补齐：

- `body_a / body_b`
- `collider_a / collider_b`
- 接触点到质心的力臂 `r_a / r_b`
- 世界空间逆惯性张量
- tangent 方向
- normal / tangent effective mass
- restitution bias
- 本帧内累计的法向与切向冲量

这里有一个关键设计：**solver contact 只是当前帧的快照**。每次 `step(dt)` 都重新收集、重新构建，不做 persistent manifold 和 warm starting。

### Effective Mass

当前实现先构造：

$$
M_{eff} =
(m_a^{-1} + m_b^{-1}) I
- [r_a]_{\times} I_a^{-1} [r_a]_{\times}
- [r_b]_{\times} I_b^{-1} [r_b]_{\times}
$$

然后再把它分别投影到法向和切向：

$$
m_n = \mathbf{n} \cdot (M_{eff}\mathbf{n}), \qquad
m_t = \mathbf{t} \cdot (M_{eff}\mathbf{t})
$$

它们就是后续法向冲量和摩擦冲量的分母。

### 代码对应关系

- `rb::RigidBodySystem::collect_contacts(...)`
- `rb::RigidBodySystem::generate_contact_from_pair(...)`
- `rb::RigidBodySystem::build_solver_contacts(...)`
- `src/rtr/system/physics/collision/*.hpp`
- `src/rtr/system/physics/rigid_body/contact.hpp`

## 速度阶段：Projected Gauss-Seidel (PGS)

当前求解器使用逐接触、逐迭代的 PGS。

### 接触点相对速度

先计算接触点相对速度：

$$
\mathbf{v}_{rel} =
(\mathbf{v}_b + \boldsymbol{\omega}_b \times \mathbf{r}_b)
- (\mathbf{v}_a + \boldsymbol{\omega}_a \times \mathbf{r}_a)
$$

### 法向冲量

如果法向 effective mass 足够大，则计算：

$$
\Delta \lambda_n =
- \frac{v_{rel,n} - b_{rest}}{m_n}
$$

这里：

- $v_{rel,n} = \mathbf{v}_{rel} \cdot \mathbf{n}$
- $b_{rest}$ 是 restitution bias

当前实现使用**累计冲量**：

- 先保存 `previous_normal_impulse_sum`
- 再算 `next_normal_impulse_sum = max(previous + delta, 0)`
- 实际施加的是两者差值 `actual_delta_normal_impulse`

这样可以自然地保证法向冲量不变成“拉力”。

### 切向摩擦冲量

法向冲量更新后，会重新读取接触点相对速度，再沿切向求一个增量摩擦冲量。累计后的切向冲量被 clamp 到：

$$
[-\mu \lambda_n,\ \mu \lambda_n]
$$

其中：

- $\mu = \sqrt{\mu_a \mu_b}$
- `normal_impulse_sum` 就是当前帧累计的法向冲量

这对应一个典型的库仑摩擦圆锥在线性化后的区间约束。

### PGS 流程图

```text
for each solver iteration:
    for each contact:
        read current body velocities
        solve normal impulse
        clamp accumulated normal impulse >= 0
        apply normal impulse
        solve tangent impulse
        clamp tangent impulse by friction limit
        apply tangent impulse
```

### 代码对应关系

- `rb::RigidBodySystem::apply_velocity_impulses(...)`
- `rb::RigidBodySystem::apply_impulse_at_contact(...)`
- `rb::RigidBodySystem::combined_restitution(...)`
- `rb::RigidBodySystem::combined_friction(...)`

## 位置阶段：穿透修正

速度求解之后，当前实现还会再做若干轮位置修正。

对每个 `SolverContact`：

- 取当前 contact snapshot 中的 penetration；
- 乘以固定比例 `0.8`，得到 `corrected_penetration`；
- 按两边 inverse mass 的比例分配位移修正；
- 更新 `contact.penetration`，但**不重建接触几何**。

它的目标不是做高精度几何约束投影，而是用一个简单、离散的 correction pass 缓解 interpenetration。

这也解释了当前实现的一个边界：

- 位置阶段使用的是“本帧先前收集到的 contact snapshot”；
- 它不会在同一帧内重新做碰撞检测；
- 因此穿透很深或几何变化很快时，效果会有限。

### 代码对应关系

- `rb::RigidBodySystem::apply_position_correction(...)`
- `rb::RigidBodySystem::step(...)`

## `collision/` 与 `rigid_body/` 的职责边界

当前目录边界是有意设计的：

- `collision/` 只负责纯几何层面的世界碰撞体和接触生成；
- `rigid_body/` 负责 body id、solver contact、冲量求解和运行时耦合状态。

这样做的好处是：

- 碰撞检测逻辑能保持几何可复用；
- 刚体求解器需要的耦合状态不会泄露到纯碰撞层。

## 当前简化与限制

当前刚体求解器仍然是一个可用但保守的 v1：

- 没有 persistent contact manifold。
- 没有 warm starting。
- 没有 continuous collision detection。
- 位置修正阶段不重建接触几何。
- solver contact 只在当前帧有效。

这些简化让实现更直接，也更容易对照课程或教科书理解，但它们都会影响高速度、高穿透或复杂堆叠场景下的稳定性。

## 关键代码入口

- `src/rtr/system/physics/rigid_body/rigid_body_system.hpp`
- `src/rtr/system/physics/rigid_body/contact.hpp`
- `src/rtr/system/physics/collision/contact.hpp`
- `src/rtr/framework/integration/physics/rigid_body_scene_sync.hpp`
