# Physics Runtime 集成

本文档说明 framework 层如何把 scene graph、刚体物理、IPC deformable runtime 和渲染侧 mesh cache 串成一条完整的 fixed-step 路径。

## 当前 fixed tick 入口

当前权威入口是 `step_scene_physics(scene, physics_system, dt)`：

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
physics_system.step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());

physics_system.ipc_system().step();
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

这里有两个关键点：

- `PhysicsSystem::step(dt)` 目前只推进刚体世界。
- IPC 的 `step()` 和 IPC -> scene 的回写，都显式放在 framework integration 层完成。

## 为什么 IPC 不放进 `PhysicsSystem::step()`

刚体子系统直接使用 runtime fixed tick 传入的 `dt`。

IPC 子系统则有自己的 `IPCConfig::dt`，内部执行的是 backward Euler 下的非线性优化求解。把 `ipc_system().step()` 留在 `scene_physics_step` 这一层，可以明确保留 deformable runtime 的固定时间步边界，同时让 scene 写回的位置也更清晰。

当前 example 里会把 `AppRuntimeConfig::fixed_delta_seconds` 设成 `0.01`，与默认的 `IPCConfig::dt` 对齐。

## Scene / Physics / Renderer 的状态所有权

| 层 | 保存什么 | 谁是运行时权威 |
| --- | --- | --- |
| Scene Graph | GameObject、层级、组件关系 | framework 层 |
| `RigidBodyWorld` | 刚体状态、碰撞体、接触 | 刚体 runtime |
| `ipc::IPCSystem` / `ipc::IPCState` | deformable 节点状态、质量、per-body offset | IPC runtime |
| `DeformableMeshComponent` | 面向渲染的表面 mesh 副本 | 渲染侧缓存 |

这里最重要的边界是：

- 刚体一旦进入模拟，动态位姿以 `RigidBodyWorld` 为准；
- deformable 一旦进入模拟，节点位置以 `IPCSystem` / `IPCState` 为准；
- `DeformableMeshComponent` 只保存写给 GPU 的表面副本，不是 deformable 运行时的权威状态。

## 刚体方向

刚体 runtime 仍然是标准的 pre-sync / solve / post-sync：

```text
Scene -> sync_scene_to_rigid_body(...) -> RigidBodyWorld
RigidBodyWorld -> sync_rigid_body_to_scene(...) -> Scene
```

动态刚体在求解后把 transform 写回 scene graph；静态和非动态对象在 pre-pass 中仍由 scene 侧驱动。

## IPC 方向

IPC 当前采用稍微不同的所有权方式：

- scene -> IPC 主要发生在注册时，而不是每帧
- IPC -> scene 会在每个 fixed tick 的 `ipc_system().step()` 之后发生

原因是当前 `TetBody` 会在初始化后注册到 `IPCSystem` 中，之后节点状态由 deformable runtime 自己维护，scene graph 不会每帧反向覆盖这些节点位置。

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

RigidBodyWorld <----- PhysicsSystem::step(dt)

IPCSystem::step()
    -> 更新 IPCState::x
    -> sync_ipc_to_scene(...)
    -> apply_deformed_surface(...)
```

当前 deformable 路径故意统一通过 `DeformableMeshComponent::apply_deformed_surface(...)` 写回。这样 CPU mesh、法线重算和 GPU 延迟上传仍然沿用现有渲染组件语义。
