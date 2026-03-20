# 物理系统总览 (Overview)

本文档描述当前 `rtr::system::physics` 的真实运行时结构。现在的 physics system 不再是“仅刚体”运行时：`PhysicsSystem` 同时持有 `RigidBodyWorld` 与 `ClothWorld`，框架层通过 `step_scene_physics()` 负责 scene graph 和两个物理 world 之间的同步。

## 当前能力与边界

当前 physics system 已经包含：

- 刚体状态积分、基础碰撞检测与碰撞响应求解。
- mass-spring cloth、pinned vertex、重力、阻尼、fixed-step substeps。
- scene graph 与 physics world 的双向同步。
- cloth 顶点回写和法线重算。

当前明确**不包含**：

- cloth collision。
- cloth self-collision。
- cloth / rigid-body coupling。
- implicit cloth solver。
- PBD / XPBD cloth。
- FEM / water runtime。

## 架构总览

1. **`PhysicsSystem`**
   当前物理运行时的总容器，内部持有一个 `RigidBodyWorld` 和一个 `ClothWorld`。它本身只负责调用 `RigidBodyWorld::step(dt)` 与 `ClothWorld::step(dt)`，不直接做 scene 同步。

2. **Framework Integration (`src/rtr/framework/integration/physics/`)**
   负责把 scene graph 中的组件状态同步到物理运行时，再把模拟结果写回 scene graph 和 renderer。权威入口是 `step_scene_physics(scene, physics_system, dt)`。

3. **`RigidBodyWorld`**
   保存刚体、附着碰撞体、接触快照和冲量求解过程。当前算法是 semi-implicit Euler 积分 + 接触生成 + PGS 速度求解 + penetration correction。

4. **`ClothWorld`**
   保存 cloth instance、spring network 与 cloth state。当前算法是显式 mass-spring + semi-implicit Euler + substeps + damping。

5. **Scene Graph / Renderer**
   scene graph 仍然是运行时对象组织和渲染可见表示；但是 dynamic rigid body 的权威位姿在 `RigidBodyWorld`，cloth 的权威顶点状态在 `ClothWorld`。

## Fixed Tick 运行数据流

当前 fixed tick 路径如下：

```text
AppRuntime fixed tick
    |
    v
step_scene_physics(scene, physics_system, dt)
    |
    +--> sync_scene_to_rigid_body(scene, rigid_body_world)
    +--> sync_scene_to_cloth(scene, cloth_world)
    +--> PhysicsSystem::step(dt)
    |       |
    |       +--> RigidBodyWorld::step(dt)
    |       \--> ClothWorld::step(dt)
    +--> sync_rigid_body_to_scene(scene, rigid_body_world)
    \--> sync_cloth_to_scene(scene, cloth_world)
```

这条路径有两个关键点：

- `PhysicsSystem::step()` 只做 world 内部求解，不负责 scene/physics 边界。
- 当前调用顺序是**先 rigid body，后 cloth**。

## 状态权威与所有权

| 位置 | 保存内容 | 谁是权威 |
| --- | --- | --- |
| Scene Graph | `GameObject`、节点层级、组件挂载、渲染可见对象 | 框架层 |
| `RigidBodyWorld` | 刚体位置/速度/朝向/角速度、collider、solver contacts | Dynamic rigid body 运行时权威 |
| `ClothWorld` | cloth 顶点位置、速度、质量、pinned mask、spring network | Cloth 运行时权威 |
| `DeformableMeshComponent` | 供 renderer 使用的当前局部顶点与法线副本 | Cloth 输出缓存，不是权威模拟状态 |

因此：

- Dynamic rigid body 开始模拟后，不应再把 scene graph transform 当成权威状态。
- Cloth 注册后，顶点运行时状态保存在 `ClothWorld`，`sync_cloth_to_scene()` 只是把结果回写到 renderer。

## 算法地图

| 子系统 | 主要状态 | 当前算法 | 主要入口 | 当前不做 |
| --- | --- | --- | --- | --- |
| Rigid Body | 刚体平移/旋转状态、碰撞体、solver contacts | semi-implicit Euler + contact generation + PGS + positional correction | `RigidBodyWorld::step()` | CCD、persistent manifold、warm starting |
| Cloth | 顶点位置/速度/质量、spring network、pinned mask | explicit mass-spring + semi-implicit Euler + substeps + spring/global damping | `build_cloth_spring_network()`、`ClothWorld::step()` | collision、自碰撞、PBD/XPBD、implicit solver |
| Framework Integration | scene graph、physics worlds、renderer mesh | fixed tick scene/physics sync | `step_scene_physics()` | 自动 cloth attach target、cloth 外部驱动同步 |

## 目录与代码入口

当前最关键的入口文件是：

- `src/rtr/system/physics/physics_system.hpp`
- `src/rtr/framework/integration/physics/scene_physics_step.hpp`
- `src/rtr/framework/integration/physics/rigid_body_scene_sync.hpp`
- `src/rtr/framework/integration/physics/cloth_scene_sync.hpp`
- `src/rtr/system/physics/rigid_body/rigid_body_world.hpp`
- `src/rtr/system/physics/cloth/cloth_world.hpp`

## 延伸阅读

- 运行时集成：`runtime-integration.zh.md`
- 布料模拟与调参：`cloth-simulation.zh.md`
- 刚体算法与碰撞响应：`rigid-body-dynamics.zh.md`
