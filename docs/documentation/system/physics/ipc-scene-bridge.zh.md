# IPC Scene Bridge

本文档说明 framework 层如何把 IPC deformable runtime 和 scene / renderer 组件连接起来。

相关代码主要在：

- `src/rtr/framework/component/physics/ipc/ipc_tet_component.hpp`
- `src/rtr/framework/integration/physics/ipc_scene_sync.hpp`
- `src/rtr/framework/integration/physics/scene_physics_step.hpp`

## 为什么需要 Scene Bridge

`IPCSystem` 在 `IPCState::x` 中保存 deformable 的权威节点状态。

但 renderer 并不直接消费体四面体状态，它消费的是 `DeformableMeshComponent` 持有的一份可渲染表面网格 `ObjMeshData`。

因此 runtime 里必须有一层桥接逻辑，负责：

1. 记住 scene 里的哪个对象对应 `IPCSystem` 中的哪个 tet body；
2. 记住 tet 边界和 render mesh 之间的映射；
3. 在每次 IPC 求解后，把新的节点位置写回 render-facing mesh。

## `IPCTetComponent`

`IPCTetComponent` 是一个挂在 scene 对象上的 IPC tet authoring/cache 组件。

它保存：

- `source_body`：用于重新注册的 authoring/source `TetBody`；
- `surface_cache`：一次性 boundary extraction 得到的 `TetSurfaceMapping`；
- `mesh_cache`：可以每帧原地更新的 `ObjMeshData`。
- 一组 dirty / lifecycle 标记，用来告诉 scene sync 哪些内容需要推到 runtime。

### 它不负责什么

`IPCTetComponent` 不拥有 render resource，但它拥有 tet 侧 source data 以及由它导出的 surface cache。

构造阶段会先导出：

```text
source TetBody
  -> build_tet_surface_mapping(body)
  -> tet_rest_to_surface_mesh(body.geometry, surface)
  -> mesh_cache
```

然后再用这份缓存出来的初始 mesh 创建渲染资源：

```text
IPCTetComponent.mesh_cache
  -> create DeformableMesh resource
  -> add DeformableMeshComponent
```

这个职责拆分很重要：

- `IPCTetComponent` 管 source tet 数据和 scene 侧缓存；
- `DeformableMeshComponent` 管理渲染侧 mesh handle；
- `sync_scene_to_ipc(...)` 负责 create / remove / update runtime body；
- `sync_ipc_to_scene(...)` 负责把 runtime 形变写回这个 mesh。

## 注册流程

当前一个 IPC tet scene 对象的最小注册流程是：

```text
1. 构造 TetBody
2. 设置 fixed vertices / material
3. 挂 IPCTetComponent(std::move(source_body))
5. 用 ipc_tet.mesh_cache() 创建 DeformableMesh resource
6. 给同一个 GameObject 挂 DeformableMeshComponent
7. 让 sync_scene_to_ipc(scene, ipc_system) 去 create 或 update runtime tet body
```

现在不需要外部手工 `add_tet_body(...)` 或 `initialize()`。只要注册发生变化，`IPCSystem` 会在下一次 `step(delta_seconds)` 前自动重建全局状态。

## `sync_ipc_to_scene(...)`

`sync_ipc_to_scene(scene, ipc_system)` 是每个 fixed tick 的 IPC -> scene 回写函数。

对每个 active object，它会：

1. 找到 `IPCTetComponent`
2. 找到同一个对象上的 `DeformableMeshComponent`
3. 如果当前 `GameObject` 没有对应 runtime tet body，就跳过
4. 用 owner id 在 `IPCSystem` 里找到对应 tet body
5. 从 `body.info.dof_offset / 3` 恢复全局顶点偏移
6. 用全局 DOF 向量原地更新 `mesh_cache`
7. 提取 positions 和 normals
8. 调 `apply_deformed_surface(...)`

## 为什么同时需要 owner lookup 和 `dof_offset`

`TetSurfaceMapping.surface_vertex_ids` 保存的是 **body-local** 顶点编号。  
而 `IPCState::x` 是所有 body 连接起来的 **global** `3N` 向量。

因此 bridge 必须先做：

$$
\text{global vertex id} = \text{body vertex offset} + \text{local surface vertex id}
$$

scene bridge 会先通过 `IPCSystem` 的 owner 映射回答“这个 `GameObject` 对应哪个 runtime body”，再通过 `dof_offset` 回答“这个 body 在当前全局状态向量里从哪里开始”。

这也是为什么 `tet_to_mesh.hpp` 的写回接口支持 `vertex_offset` 参数。

## `scene_physics_step(...)` 中的位置

framework 层当前的 fixed-step 顺序已经变成：

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_system());
sync_scene_to_ipc(scene, physics_system.ipc_system());

physics_system.rigid_body_system().step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_system());

physics_system.ipc_system().step(dt);
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

刚体路径仍然是标准的 pre-sync / solve / post-sync。

IPC 现在也走 pre-sync / solve / post-sync，只是 scene -> IPC 这边仍然只同步 source body / material / cache 变化，而不是每帧 scene node transform。

## 当前边界

这层 bridge 刻意保持很薄：

- 注册生命周期现在由 component 驱动，但仍然默认 tet 拓扑不会每帧修改；
- `IPCTetComponent` 本身不再持有 `IPCSystem&` 或 runtime body handle；
- 不支持每帧把 scene transform 反向写回 deformable 节点；
- 还没有接 contact object 或 obstacle body；
- 约定 `IPCTetComponent` 和 `DeformableMeshComponent` 出现在同一个 `GameObject` 上。

对于当前 deformable runtime 和 fixed-end example，这已经足够，而且职责边界很清晰。
