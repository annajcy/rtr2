# 物理系统总览

RTR2 当前在 `src/rtr/system/physics/` 下有两个方向：

- 已接入运行时的刚体子系统 `rigid_body/`
- 正在建设中的 IPC/FEM 子系统 `ipc/`

当前 `PhysicsSystem` 只持有 `RigidBodyWorld`。`ipc/` 子树已经有第一批核心数据结构，但还没有接到运行时主循环里。

## 当前范围

- 已实现：刚体模拟、碰撞检测与响应、scene/physics 同步、IPC/FEM 的基础数据结构。
- 未实现：cloth runtime、IPC 求解循环、IPC contact/barrier/CCD、rigid-body/IPC coupling。

## Fixed Tick 流程

```text
step_scene_physics(scene, physics_system, dt)
    -> sync_scene_to_rigid_body(...)
    -> PhysicsSystem::step(dt)
         -> RigidBodyWorld::step(dt)
    -> sync_rigid_body_to_scene(...)
```

`PhysicsSystem::step()` 只负责 world 内部求解。scene 同步仍然放在 framework integration 层。IPC 子系统目前还没有进入这条 fixed tick 路径。

## 运行时所有权

| 位置 | 保存内容 | 谁是运行时权威 |
| --- | --- | --- |
| Scene Graph | GameObject、层级、渲染相关组件 | framework 层 |
| `RigidBodyWorld` | 刚体状态、碰撞体、接触和求解状态 | 刚体运行时 |
| `ipc::IPCState` | 未来 deformable 的全局节点状态 | 未来 deformable 运行时 |

一旦动态刚体开始模拟，scene transform 就不应再被当成动态状态的权威来源。未来 deformable runtime 建起来以后，也会沿用同样的状态所有权划分。

## IPC 文档子树

IPC 的详细文档已经拆到与源码镜像的 [`documentation/system/physics/ipc/`](ipc/overview.md) 子树中。

- [`ipc/overview.md`](ipc/overview.md)：目录职责和当前能力边界
- [`ipc/core/ipc_state.md`](ipc/core/ipc_state.md)：全局 `3N` 状态布局和数学公式
- [`ipc/model/ipc_body.md`](ipc/model/ipc_body.md)：body 类型和全局 DOF 映射元数据
- [`ipc/model/tet_body.md`](ipc/model/tet_body.md)：`TetGeometry`、`TetBody`、rest-shape 预计算和 block 生成工具
- [`ipc/model/tet_mesh_convert.md`](ipc/model/tet_mesh_convert.md)：tet surface 提取和 tet 到渲染网格的写回
- [`ipc/model/obstacle_body.md`](ipc/model/obstacle_body.md)：当前 obstacle 占位类型的语义

这棵子树严格跟随源码目录：每个 `ipc/*.hpp` 都有自己的页面，每个目录都有自己的 `overview.md`。

## 建议阅读顺序

如果你关心已经接入运行时的物理路径，继续看：

- [`runtime-integration.md`](runtime-integration.md)
- [`rigid-body-dynamics.md`](rigid-body-dynamics.md)

如果你关心 deformable 数据层，继续看：

- [`ipc/overview.md`](ipc/overview.md)
