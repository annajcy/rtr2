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

`IPCTetComponent` 是一个挂在 scene 对象上的 IPC tet 桥接缓存组件。

它保存：

- `body_index`：这个对象对应 `IPCSystem` 中的哪个 body；
- `surface_cache`：一次性 boundary extraction 得到的 `TetSurfaceResult`；
- `mesh_cache`：可以每帧原地更新的 `ObjMeshData`。

### 它不负责什么

`IPCTetComponent` **不负责**自己生成初始 render mesh。

初始 render mesh 的来源仍然是：

```text
TetBody
  -> extract_tet_surface(body)
  -> tet_to_mesh(body.geometry, surface)
  -> initial ObjMeshData
```

只有在这份初始 mesh 已经存在之后，才会继续做：

```text
initial ObjMeshData
  -> create DeformableMesh resource
  -> add DeformableMeshComponent
  -> add IPCTetComponent(body_index, surface, mesh_cache)
```

这个职责拆分很重要：

- `TetBody` 提供体网格源数据；
- `DeformableMeshComponent` 管理渲染侧 mesh handle；
- `IPCTetComponent` 只保存桥接元数据和缓存。

## 注册流程

当前一个 IPC tet scene 对象的最小注册流程是：

```text
1. 构造 TetBody
2. 设置 fixed vertices / material
3. extract_tet_surface(body)
4. tet_to_mesh(body.geometry, surface)
5. 用初始 mesh 创建 DeformableMesh resource
6. 给 GameObject 挂 DeformableMeshComponent
7. 把 TetBody 注册进 IPCSystem
8. 挂 IPCTetComponent(body_index, surface, mesh_cache)
9. 调 ipc_system.initialize()
```

这里 `body_index` 是这条桥上的稳定 key。它让 scene 对象后续可以通过 `ipc_system.tet_body(body_index).info.dof_offset` 找回该 body 在全局状态向量中的起始顶点偏移。

## `sync_ipc_to_scene(...)`

`sync_ipc_to_scene(scene, ipc_system)` 是每个 fixed tick 的 IPC -> scene 回写函数。

对每个 active object，它会：

1. 找到 `IPCTetComponent`
2. 找到同一个对象上的 `DeformableMeshComponent`
3. 在 `IPCSystem` 里找到对应的 tet body
4. 从 `body.info.dof_offset / 3` 恢复全局顶点偏移
5. 用全局 DOF 向量原地更新 `mesh_cache`
6. 提取 positions 和 normals
7. 调 `apply_deformed_surface(...)`

## 为什么 `body_index` 不能省略

`TetSurfaceResult.surface_vertex_ids` 保存的是 **body-local** 顶点编号。  
而 `IPCState::x` 是所有 body 连接起来的 **global** `3N` 向量。

因此 bridge 必须先做：

$$
\text{global vertex id} = \text{body vertex offset} + \text{local surface vertex id}
$$

这也是为什么当前 `tet_mesh_convert.hpp` 的写回接口支持 `vertex_offset` 参数。

如果没有这个 offset，第二个、第三个 body 在回写时就会错误地从全局状态向量的 body 0 区间读取位置。

## `scene_physics_step(...)` 中的位置

framework 层当前的 fixed-step 顺序已经变成：

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
physics_system.step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());

physics_system.ipc_system().step();
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

刚体路径仍然是标准的 pre-sync / solve / post-sync。

IPC 路径则稍微不同：

- scene -> IPC 主要发生在注册时，而不是每帧；
- IPC -> scene 在每次 `ipc_system().step()` 后发生。

## 当前边界

这层 bridge 刻意保持很薄：

- 默认假设 `TetBody` 在初始化后静态注册；
- 不支持每帧把 scene transform 反向写回 deformable 节点；
- 还没有接 contact object 或 obstacle body；
- 约定 `IPCTetComponent` 和 `DeformableMeshComponent` 出现在同一个 `GameObject` 上。

对于当前 deformable runtime 和 fixed-end example，这已经足够，而且职责边界很清晰。
