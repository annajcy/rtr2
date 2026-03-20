# Cloth Simulation

本文档说明当前 RTR2 中已经实现的 cloth runtime：一个运行在三角网格上的显式 mass-spring solver。本文只描述当前代码里真实存在的行为，不扩展到尚未实现的 cloth collision、self-collision、PBD/XPBD 或 implicit solver。

## 当前实现边界

当前 cloth runtime 支持：

- 任意三角网格输入。
- 从 mesh 拓扑自动构建 edge spring 与 bend spring。
- uniform vertex mass。
- gravity、spring damping、global velocity damping。
- fixed-step substeps。
- pinned vertices。
- deformable mesh 顶点和法线回写。

当前 cloth runtime 不支持：

- cloth collision。
- cloth self-collision。
- cloth / rigid-body coupling。
- stretch limiting / strain limiting。
- implicit cloth、PBD、XPBD。

## 数据流与核心类型

当前 cloth 数据流如下：

```text
DeformableMeshComponent
    |
    \--> build_cloth_topology(...)
            |
            +--> ClothTopology
            +--> ClothState::from_topology(...)
            \--> ClothWorld::create_cloth(...)
                     |
                     \--> build_cloth_spring_network(...)
                             |
                             \--> ClothSpringNetwork

fixed tick:
ClothWorld::step(dt)
    |
    \--> update ClothState.positions / velocities
            |
            \--> sync_cloth_to_scene(...)
                    |
                    +--> recompute_vertex_normals(...)
                    \--> DeformableMeshComponent::apply_deformed_surface(...)
```

关键类型如下：

| 类型 | 作用 |
| --- | --- |
| `ClothTopology` | 保存 rest positions、triangles、unique edges、edge rest lengths、渲染三角形索引 |
| `ClothSpringKind` | 当前只有 `Edge` 和 `Bend` 两种弹簧类型 |
| `ClothSpring` | 保存一根弹簧的类型、两个端点顶点、rest length |
| `ClothSpringNetwork` | cloth 注册时构建好的 spring 缓存 |
| `ClothState` | 保存 `positions / velocities / masses / pinned_mask` |
| `ClothParams` | solver 的可调参数集合 |
| `ClothInstance` | `topology + state + params + spring_network` |
| `ClothWorld` | cloth runtime 容器，负责 step 与 instance 管理 |

### 代码对应关系

- `src/rtr/system/physics/cloth/cloth_topology.hpp`
- `src/rtr/system/physics/cloth/cloth_spring.hpp`
- `src/rtr/system/physics/cloth/cloth_state.hpp`
- `src/rtr/system/physics/cloth/cloth_world.hpp`
- `src/rtr/framework/component/physics/cloth/cloth_component.hpp`

## 弹簧网络构建

### Edge Springs

每条 unique mesh edge 都会生成一根 `Edge` spring。它的端点直接来自 `ClothTopology::edges`，rest length 复用 `topology.edge_rest_lengths`。

因此当前实现并不再把“几何边”和“物理弹簧”混在同一个概念里，而是通过 `ClothSpringNetwork` 明确缓存 solver 实际要遍历的 spring 列表。

### Bend Springs

对于每条 **interior edge**：

- 先找到这条 edge 两侧相邻的两个 triangle；
- 再取出每个 triangle 中不属于该 edge 的 opposite vertex；
- 用这两个 opposite vertices 之间的 rest distance 构造一根 `Bend` spring。

对于 boundary edge，不会生成 bend spring。

### 构建伪代码

```text
for each unique edge (va, vb):
    add Edge spring(va, vb, edge_rest_length)

    if edge is boundary:
        continue

    triangle_a, triangle_b = adjacent triangles
    opposite_a = opposite vertex of triangle_a against (va, vb)
    opposite_b = opposite vertex of triangle_b against (va, vb)
    bend_rest_length = |rest_position(opposite_b) - rest_position(opposite_a)|
    add Bend spring(opposite_a, opposite_b, bend_rest_length)
```

### 代码对应关系

- `build_cloth_spring_network(...)`
- `detail::opposite_triangle_vertex(...)`
- 文件：`src/rtr/system/physics/cloth/cloth_spring.hpp`

## 状态变量与 pinned vertex

`ClothState` 当前保存四组数组：

- `positions`：当前顶点位置。
- `velocities`：当前顶点速度。
- `masses`：每个顶点的质量；目前默认是 uniform mass。
- `pinned_mask`：当前顶点是否被固定。

初始状态通过 `ClothState::from_topology(...)` 创建：

- `positions = topology.rest_positions`
- `velocities = 0`
- `masses = default_vertex_mass`
- `pinned_mask = 0`

在 `ClothComponent::on_enable()` 中，传入的 pinned vertex 会被标记为 true。

当前 pinned vertex 的语义非常明确：

- 它们在积分阶段会被跳过；
- 每个 substep 结束时会被强制写回 `topology.rest_positions`；
- 对应速度会被清零。

这意味着 pinned target 不是可动画的 attach target，而是注册时 rest mesh 上的固定位置。

### 代码对应关系

- `ClothState::from_topology(...)`
- `ClothState::set_pinned(...)`
- `ClothWorld::enforce_pinned_vertices(...)`

## 求解算法

### 外层时间步与 substeps

`ClothWorld::step(dt)` 不直接用完整 `dt` 做一次积分，而是先计算：

$$
h = \frac{\Delta t}{\text{substeps}}
$$

然后对每个 cloth instance 连续执行 `substeps` 次更小的 substep。这样做的目的，是在保留显式求解器简单性的同时，尽量提高稳定性。

### 单根弹簧的力

设一根弹簧连接顶点 $a$ 和 $b$，则当前实现使用：

$$
\mathbf{d} = \mathbf{x}_b - \mathbf{x}_a,\qquad
\ell = \|\mathbf{d}\|,\qquad
\mathbf{n} = \frac{\mathbf{d}}{\ell}
$$

$$
e = \ell - \ell_0
$$

$$
v_{rel} = (\mathbf{v}_b - \mathbf{v}_a) \cdot \mathbf{n}
$$

$$
\mathbf{F}_{a} = (k e + c v_{rel}) \mathbf{n},\qquad
\mathbf{F}_{b} = -\mathbf{F}_{a}
$$

其中：

- $k$ 来自 `edge_stiffness` 或 `bend_stiffness`；
- $c$ 来自 `spring_damping`；
- $\ell_0$ 是该弹簧的 rest length。

如果当前长度不是 finite，或者长度过小，当前实现会直接跳过这根 spring，避免除零和数值异常。

### 单个 substep 的精确顺序

当前实现采用 semi-implicit Euler，substep 内部顺序固定为：

```text
forces = 0

for each unpinned vertex:
    forces += mass * gravity

for each spring:
    compute direction, extension, relative speed
    forces[a] += spring_force
    forces[b] -= spring_force

for each unpinned vertex:
    velocity += (force / mass) * h
    velocity *= max(0, 1 - velocity_damping * h)
    position += velocity * h

for each pinned vertex:
    position = rest_position
    velocity = 0
```

这里包含三层稳定化机制：

1. **spring damping**
   沿弹簧方向抑制相对速度，减少拉伸/压缩振荡。

2. **global velocity damping**
   在速度更新后统一乘以 `max(0, 1 - velocity_damping * h)`，快速压住全局余振。

3. **substeps**
   用更小的时间步长执行显式积分，降低爆炸风险。

### 为什么使用 semi-implicit Euler

与显式欧拉相比，semi-implicit Euler 会先更新速度，再用更新后的速度推进位置：

$$
\mathbf{v}^{k+1} = \mathbf{v}^k + \frac{\mathbf{F}^k}{m} h
$$

$$
\mathbf{x}^{k+1} = \mathbf{x}^k + \mathbf{v}^{k+1} h
$$

这仍然是显式方法，但通常比最朴素的 explicit Euler 更稳定，尤其适合当前这种“实现简单、易调试、能配合 substeps”的 cloth v1。

它的代价也很明显：

- 对 `stiffness / mass / dt` 比例依然敏感；
- stiffness 太高时仍然容易爆炸；
- 需要靠 substeps 和 damping 才能进入稳定区间。

### 代码对应关系

- `ClothWorld::step(...)`
- `ClothWorld::step_instance(...)`
- `ClothWorld::stiffness_for(...)`
- 文件：`src/rtr/system/physics/cloth/cloth_world.hpp`

## 稳定性与数值行为

当前 cloth solver 是一个显式 mass-spring 系统，因此它最敏感的比例是：

- stiffness 太高；
- mass 太小；
- 单帧步长太大；
- damping 太低。

这些量组合起来，很容易让速度和位移在一个或几个 substep 内快速放大，表现为：

- 顶点突然飞走；
- 布料“抽搐”或高频振荡；
- NaN / Inf；
- 布片像口香糖一样被拉长。

当前实现依赖以下手段稳定：

- 把完整 `dt` 切成多个 substeps；
- 用 spring damping 压缩局部相对运动；
- 用 velocity damping 压缩全局速度；
- 对 pinned 顶点每个 substep 都强制回到 rest positions。

这可以让一个 v1 solver 跑起来，但它并不能替代 implicit solver、strain limiting 或 PBD/XPBD 的稳定性。

## 参数说明

`ClothParams` 的当前默认值如下：

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `default_vertex_mass` | `1.0f` | 所有顶点的默认质量 |
| `gravity` | `(0, -9.81, 0)` | 重力加速度 |
| `edge_stiffness` | `800.0f` | 面内 edge spring 的刚度 |
| `bend_stiffness` | `80.0f` | 抗弯 bend spring 的刚度 |
| `spring_damping` | `8.0f` | 沿 spring 方向的相对速度阻尼 |
| `velocity_damping` | `0.02f` | 每个 substep 的全局速度阻尼 |
| `substeps` | `8` | 一个外层 `dt` 被切分的子步数 |

### 如何理解这些参数

- `default_vertex_mass`
  目前是 uniform mass，不按面积或质量密度分配。调小它会让同样的力产生更大的加速度。

- `edge_stiffness`
  决定布在面内被拉伸时有多“硬”。太低会像橡胶膜，太高在显式求解器里会更难稳定。

- `bend_stiffness`
  决定布料抗折叠、抗弯曲的程度。它通常应明显低于 `edge_stiffness`，否则整体会更像弹性壳。

- `spring_damping`
  主要抑制沿弹簧方向的相对运动。太低会反复弹，太高会显得沉闷、动作发粘。

- `velocity_damping`
  是一个更粗粒度的全局减振器，用来快速压制整体余振。

- `substeps`
  不是“更高精度”的抽象选项，而是当前显式 solver 的核心稳定性旋钮。

## 调参与排错

| 症状 | 常见原因 | 调整方向 |
| --- | --- | --- |
| 一开始就瞬间塌陷 | `edge_stiffness` 太低，或 `gravity` 太大，或 pinned 约束不足 | 提高 `edge_stiffness`，降低 `gravity`，检查 pinned vertices |
| 上下持续振荡 | `spring_damping`、`velocity_damping` 太低 | 先提高 `spring_damping`，再少量提高 `velocity_damping` |
| 模拟直接炸掉 | stiffness 太高、mass 太小、`dt / substeps` 太大 | 降低 `edge_stiffness` / `bend_stiffness`，提高 `default_vertex_mass`，提高 `substeps` |
| 像口香糖一样过软 | `edge_stiffness` 太低，或者 `bend_stiffness` 与 `edge_stiffness` 比例不对 | 提高 `edge_stiffness`，保持 `bend_stiffness` 明显低于 `edge_stiffness` |
| bunny 看起来不像衣服 | bunny 是闭合网格，更像 elastic shell；当前 solver 也没有 collision 和应变限制 | 把它当成 soft shell / elastic shell 示例，而不是服装布料示例 |

一个常见误区是“把质量调得越轻越像布”。对当前显式 mass-spring solver 来说，质量过小通常只会更容易炸，而不是自动更像布料。

## 使用方式

当前 cloth 的最小使用方式是：

```cpp
auto& cloth_go = scene.create_game_object("cloth");
cloth_go.add_component<DeformableMeshComponent>(resource_manager, mesh_handle, base_color);
cloth_go.add_component<ClothComponent>(
    runtime.physics_system().cloth_world(),
    pinned_vertices,
    cloth_params);
```

其中：

- `ClothComponent` 依赖 `DeformableMeshComponent`；
- pinned 顶点索引必须对当前 mesh 有效；
- cloth 运行时更新依赖 fixed tick 中的 `step_scene_physics(...)`。

### 示例

- `examples/games103_lab/lab2_cloth/lab2_cloth.cpp`
  规则 cloth patch 示例，适合观察下垂、振荡和调参行为。

- `examples/editor/cloth_bunny_editor.cpp`
  闭合 bunny mesh 示例，更接近 elastic shell 演示，不应被当成“真实衣服布料”示例。

## 当前限制

- cloth 在 local mesh space 中模拟。
- `sync_scene_to_cloth()` 当前不驱动 runtime 顶点状态。
- pinned 顶点目标是 `rest_positions`，不是动画 attach target。
- 当前没有 collision，所以“地面”“人体”“布料自碰撞”都不会真正约束 cloth。
