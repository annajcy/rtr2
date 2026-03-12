# 物理系统总览 (Overview)

本文档描述 `rtr2` 当前的 `rtr::system::physics` 架构。现阶段这套物理系统是一个“仅刚体”的运行时：`PhysicsSystem` 负责总调度，`RigidBodyWorld` 负责实际模拟，`collision` 模块负责共享的几何接触检测。

## 架构总览

当前物理模块可以分为以下几个层次：

1. **`PhysicsSystem`（运行时总调度器）**
   - **职责**：作为 `AppRuntime` 当前持有的物理运行时入口。
   - **交互流程**：`PhysicsSystem::step(scene, dt)` 会先把框架层的位姿和碰撞体状态同步进刚体世界，再调用 `RigidBodyWorld::step(dt)`，最后把动态刚体的结果回写到 scene graph。
2. **`rigid_body_scene_sync.hpp`（场景到刚体系统的适配层）**
   - **职责**：连接 `GameObject` 组件与刚体模拟状态。
   - **输入同步**：把 scene graph 中的 world transform、scale、collider 参数同步到 `RigidBodyWorld`。
   - **输出同步**：在 fixed tick 结束后，把 `RigidBodyWorld` 中 Dynamic 刚体的位姿写回 scene graph。
3. **`RigidBodyWorld`（核心模拟世界）**
   - **职责**：维护全部刚体和附着在刚体上的碰撞体，负责运动积分、接触收集与碰撞响应求解。
   - **核心管线 (`step`)**：每次 fixed step 中，先积分 Dynamic 刚体，再构建一份仅在当前帧有效的 contact snapshot，随后执行速度阶段的 PGS 迭代和位置阶段的穿透修正，最后清空外力累加器。
   - **范围边界**：当前这个 world 只管理 rigid body，不包含 cloth / FEM / water 的运行时。
4. **`RigidBody` (刚体数据容器)**
   - **职责**：保存刚体的运动状态（`RigidBodyState`）、摩擦和恢复系数、sleep/awake 状态、质量属性，以及力/力矩累加器。
4. **`collision/`（几何接触层）**
   - **职责**：提供纯碰撞检测相关的数据结构和两两碰撞接触生成逻辑。
   - **包含内容**：`ColliderShape`、`WorldCollider`、`ContactResult`、`ContactPairTrait<...>::generate(...)`。
   - **边界定义**：这一层不持有刚体 id，也不持有求解阶段的 contact cache；这些耦合类型放在 `rigid_body/` 内。

## 当前目录结构

```text
src/rtr/system/physics/
  physics_system.hpp
  common/
    physics_ids.hpp
    physics_material.hpp
    physics_step_context.hpp
  collision/
    collider_shape.hpp
    contact.hpp
    sphere_sphere.hpp
    sphere_box.hpp
    sphere_plane.hpp
    box_box.hpp
    box_plane.hpp
    mesh_plane.hpp
    plane_common.hpp
  rigid_body/
    rigid_body.hpp
    rigid_body_type.hpp
    collider.hpp
    contact.hpp
    rigid_body_world.hpp
```

## Fixed Tick 运行契约

当前运行时的 fixed-step 路径是：

1. `AppRuntime` 累积时间并执行 fixed updates。
2. 每个 fixed tick 调用 `PhysicsSystem::step(scene, fixed_dt)`。
3. `sync_scene_to_rigid_body(scene, rigid_body_world)` 同步输入。
4. `RigidBodyWorld::step(fixed_dt)` 执行积分和碰撞响应。
5. `sync_rigid_body_to_scene(scene, rigid_body_world)` 将 Dynamic 刚体结果写回 scene。

因此，Dynamic 刚体的权威模拟状态保存在 `RigidBodyWorld` 中，而 scene graph 仍然是渲染和编辑器侧可见的表示。

## `collision` 与 `rigid_body` 的职责边界

当前边界是有意这样划分的：

- `collision/` 只负责纯几何和接触生成。
- `rigid_body/` 负责 `Collider`、`Contact`、`SolverContact` 以及冲量求解。

这样做的目的，是让碰撞对检测逻辑保持可复用，同时把刚体 id、累计冲量和求解阶段的状态留在真正使用它们的 rigid-body 模块内部。
