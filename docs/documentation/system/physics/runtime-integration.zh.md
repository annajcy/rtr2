# Physics Runtime 集成

本文档说明框架层如何把 scene graph、刚体世界和 cloth 世界连接成一个完整的 fixed-step 运行时。

## 总流程

当前运行时的统一入口是 `step_scene_physics(scene, physics_system, dt)`：

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
sync_scene_to_cloth(scene, physics_system.cloth_world());
physics_system.step(dt);  // 先 rigid body，再 cloth
sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());
sync_cloth_to_scene(scene, physics_system.cloth_world());
```

这段调用顺序定义了当前 physics runtime 的边界：

- 输入同步发生在 step 之前。
- world 内部求解由 `PhysicsSystem::step()` 调度。
- 输出回写发生在 step 之后。

## 运行时数据流

```text
GameObject / Components
    |
    +--> RigidBody / Collider components
    |        |
    |        \--> sync_scene_to_rigid_body(...)
    |
    \--> DeformableMeshComponent + ClothComponent
             |
             \--> sync_scene_to_cloth(...)

RigidBodyWorld <----- PhysicsSystem::step(dt) -----> ClothWorld
    |                                                |
    \--> sync_rigid_body_to_scene(...)               \--> sync_cloth_to_scene(...)
                                                      \--> apply_deformed_surface(...)
```

## Scene、Physics、Renderer 的状态边界

| 模块 | 当前职责 | 典型状态 |
| --- | --- | --- |
| Scene Graph | 对象层级、组件组织、静态配置 | 节点 transform、组件开关、资源句柄 |
| `RigidBodyWorld` | 刚体运行时状态与碰撞求解 | 动态刚体位置/速度、碰撞体、solver contacts |
| `ClothWorld` | cloth 运行时状态与 spring 求解 | 顶点位置/速度/质量、pinned mask、spring network |
| Renderer / `DeformableMeshComponent` | 渲染使用的 mesh 副本 | 当前 deform 后顶点、重算法线 |

核心原则是：

- dynamic rigid body 的权威位姿保存在 `RigidBodyWorld`；
- cloth 的权威顶点状态保存在 `ClothWorld`；
- scene graph 和 renderer 保存的是可见表示与桥接数据，不是 cloth 顶点积分的权威来源。

## Rigid Body 集成

### 组件注册

- `RigidBody::on_enable()` 会根据 owner 当前的 world position、world rotation、world scale 创建一个 physics body。
- 各种 collider 组件的 `on_enable()` 会向 `RigidBodyWorld` 注册附着在该 body 上的 collider 记录。

这意味着刚体和碰撞体的物理世界记录，是在组件 enable 生命周期里建立的，而不是每帧临时创建的。

### 输入同步：`sync_scene_to_rigid_body(...)`

当前 pre-pass 做这些事情：

- 更新 scene graph world transforms。
- 把每个 active `RigidBody` 对应的 `scale` 同步到 physics body。
- 对于非 `Dynamic` 刚体，把 scene graph 的 position / orientation 推到 physics body。
- 把 collider 组件的局部形状参数、局部位姿、局部缩放同步到 physics collider。

这意味着：

- `Static` 和 `Kinematic` body 由 scene graph 驱动；
- `Dynamic` body 的平移和旋转在物理运行后由 solver 决定，pre-pass 不会用 scene graph 结果覆盖它们。

### 输出回写：`sync_rigid_body_to_scene(...)`

当前 post-pass 只回写 `Dynamic` body：

- `position`
- `orientation`
- `scale`

回写完成后会再次更新 scene graph world transforms，保证渲染和编辑器读到的是最新节点状态。

### 代码对应关系

- `src/rtr/framework/component/physics/rigid_body/rigid_body.hpp`
- `src/rtr/framework/component/physics/rigid_body/*.hpp`
- `src/rtr/framework/integration/physics/rigid_body_scene_sync.hpp`

## Cloth 集成

### 组件注册

`ClothComponent::on_enable()` 是当前 cloth runtime 的注册入口。它会：

1. 要求 owner 上存在 `DeformableMeshComponent`；
2. 从 deformable mesh 读取 local vertices 与 triangle indices；
3. 调用 `build_cloth_topology(...)` 构建 `ClothTopology`；
4. 用 `ClothState::from_topology(...)` 建立初始 positions / velocities / masses / pinned mask；
5. 应用 pinned vertices；
6. 通过 `ClothWorld::create_cloth(...)` 注册到 `ClothWorld`。

`ClothWorld::create_cloth(...)` 内部会进一步构建 `ClothSpringNetwork`。

### 输入同步：`sync_scene_to_cloth(...)`

当前版本的 `sync_scene_to_cloth(...)` 是一个**显式边界钩子**，但暂时不修改 cloth runtime 顶点状态。它保留这个 pre-pass 的原因是：

- scene/physics 的职责边界保持明确；
- 未来可以在这里加入校验、配置同步或 attach target 同步；
- 当前 cloth 运行时状态在注册后由 `ClothWorld` 独占管理。

换句话说，现在的 cloth pre-pass 是 no-op boundary，而不是驱动 cloth 位置的入口。

### 输出回写：`sync_cloth_to_scene(...)`

cloth post-pass 会：

1. 取出 `ClothWorld` 中的当前局部顶点位置；
2. 根据 `render_triangle_indices` 重算 vertex normals；
3. 调用 `DeformableMeshComponent::apply_deformed_surface(...)` 把顶点和法线写回渲染组件。

因此 renderer 看到的是 cloth solver 的输出，而不是 scene graph 自己维护的一份独立顶点状态。

### 代码对应关系

- `src/rtr/framework/component/physics/cloth/cloth_component.hpp`
- `src/rtr/framework/integration/physics/cloth_scene_sync.hpp`
- `src/rtr/system/physics/cloth/cloth_world.hpp`

## 常见约束与坑点

### 1. `PhysicsSystem::step()` 不做 scene 同步

如果只调用 `PhysicsSystem::step(dt)`，你只会推进两个 physics world，不会发生 scene 输入同步和输出回写。完整运行时入口应当是 `step_scene_physics(...)`。

### 2. Cloth 当前在 local mesh space 中模拟

注册时使用的是 `DeformableMeshComponent` 的 local vertices。注册之后，cloth 顶点状态由 `ClothWorld` 维护；当前实现没有一个“外部动画 attach target -> cloth 顶点”的同步通道。

### 3. Cloth owner transform 默认视为稳定

当前实现没有把 owner 的后续 transform 变化重新烘焙进 cloth rest state 或 pinned target。局部顶点仍在最初注册的 cloth 空间中求解。

### 4. 法线是每帧重算的

cloth 输出不仅更新顶点位置，还会重算 normals。这也是 cloth scene sync 和 rigid-body scene sync 的一个关键差异。
