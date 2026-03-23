# Physics Runtime 集成

本文档说明 framework 层如何把 scene graph、刚体物理、IPC deformable runtime 和渲染侧 mesh cache 串成一条完整的 fixed-step 路径。

## 当前 fixed tick 入口

当前权威入口是 `step_scene_physics(scene, physics_system, dt)`：

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_system());
sync_scene_to_ipc(scene, physics_system.ipc_system());

physics_system.rigid_body_system().step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_system());

physics_system.ipc_system().step(dt);
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

这里有两个关键点：

- 刚体和 IPC 的 step 都显式写在 framework integration 层；
- scene -> runtime 的同步发生在求解前，runtime -> scene 的同步发生在求解后。

## 为什么现在由 framework 显式推进两个子系统

刚体子系统直接使用 runtime fixed tick 传入的 `dt`。

IPC 子系统现在也直接消费 runtime fixed tick 传入的 `dt`，内部执行的是 backward Euler 下的非线性优化求解。把 `rigid_body_system().step(dt)` 和 `ipc_system().step(dt)` 都显式留在 `scene_physics_step` 这一层，可以更清楚地保留 fixed-step 边界，也让 pre-sync / post-sync 的顺序更直接。

当前 example 里通常会把 `AppRuntimeConfig::fixed_delta_seconds` 设成 `0.01`，这也比较适合 deformable 求解器设置。

## Scene / Physics / Renderer 的状态所有权

| 层 | 保存什么 | 谁是运行时权威 |
| --- | --- | --- |
| Scene Graph | GameObject、层级、组件关系 | framework 层 |
| `rb::RigidBodySystem` | 刚体状态、碰撞体、接触 | 刚体 runtime |
| `ipc::IPCSystem` / `ipc::IPCState` | deformable 节点状态、质量、per-body offset | IPC runtime |
| `DeformableMeshComponent` | 面向渲染的表面 mesh 副本 | 渲染侧缓存 |

这里最重要的边界是：

- 刚体一旦进入模拟，动态位姿以 `rb::RigidBodySystem` 为准；
- deformable 一旦进入模拟，节点位置以 `ipc::IPCSystem` / `ipc::IPCState` 为准；
- `DeformableMeshComponent` 只保存写给 GPU 的表面副本，不是 deformable 运行时的权威状态。

## 刚体方向

刚体 runtime 仍然是标准的 pre-sync / solve / post-sync：

```text
Scene -> sync_scene_to_rigid_body(...) -> rb::RigidBodySystem
rb::RigidBodySystem -> sync_rigid_body_to_scene(...) -> Scene
```

动态刚体在求解后把 transform 写回 scene graph；静态和非动态对象在 pre-pass 中仍由 scene 侧驱动。

## IPC 方向

IPC 当前仍然采用稍微不同的所有权方式：

- scene -> IPC 是 dirty 驱动的 source 同步，而不是每帧 scene transform 覆盖
- IPC -> scene 会在每个 fixed tick 的 `ipc_system().step(dt)` 之后发生

原因是当前 `TetBody` 一旦注册到 `IPCSystem`，节点状态就由 deformable runtime 自己维护，scene graph 不会每帧反向覆盖这些节点位置。

这条桥接链上的关键对象是：

- `IPCTetComponent`
- `sync_ipc_to_scene(...)`
- `DeformableMeshComponent`

更细的注册和回写流程见：

- [`ipc-scene-bridge.md`](ipc-scene-bridge.md)
- [`ipc-fixed-end-block-example.md`](ipc-fixed-end-block-example.md)

## 当前运行时数据流

```text
GameObject / Components
    |
    +--> RigidBody / Collider components
    |        |
    |        \--> sync_scene_to_rigid_body(...)
    |
    \--> DeformableMeshComponent + IPCTetComponent
             |
             \--> sync_ipc_to_scene(...)

rb::RigidBodySystem::step(dt)

ipc::IPCSystem::step(dt)
    -> 更新 IPCState::x
    -> sync_ipc_to_scene(...)
    -> apply_deformed_surface(...)
```

当前 deformable 路径故意统一通过 `DeformableMeshComponent::apply_deformed_surface(...)` 写回。这样 CPU mesh、法线重算和 GPU 延迟上传仍然沿用现有渲染组件语义。
