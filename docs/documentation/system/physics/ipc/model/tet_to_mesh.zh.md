# `tet_to_mesh.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp` 保存 tet surface 提取和渲染网格写回路径。

## 当前职责

这个文件负责 `TetGeometry / TetBody / IPCState::x -> ObjMeshData` 方向。

公开 API 包括：

- `TetSurfaceMapping`
- `build_tet_surface_mapping(...)`
- `tet_dofs_to_surface_mesh(...)`
- `tet_rest_to_surface_mesh(...)`
- `update_surface_mesh_from_tet_dofs(...)`

这也是当前 `IPCTetComponent` 和 `sync_ipc_to_scene(...)` 使用的运行时桥接层。

## `TetSurfaceMapping`

表面提取仍然遵循原来的拓扑规则：

$$
\text{surface face} \iff \text{这个三角面只出现一次}
$$

每个 tet 提供 4 个三角面；只出现一次的面就是边界面。

提取结果保存在：

- `surface_indices`：边界三角面索引
- `surface_vertex_ids`：所有参与边界面的唯一 tet 顶点编号

`surface_vertex_ids` 仍然是 body-local 的 tet 顶点 id。

## 表面网格构造

当前有两条生成 render mesh 的路径：

- `tet_rest_to_surface_mesh(...)`：从 `TetGeometry::rest_positions` 读取位置
- `tet_dofs_to_surface_mesh(...)`：从 `3N` DOF 向量读取位置

这两条路径都会：

- 压缩表面顶点集合
- 把三角面索引重写成 surface-local 编号
- 重新计算顶点法线

当 DOF 向量是多个 body 拼接后的全局状态时，`vertex_offset` 用来把 body-local surface id 平移到正确的全局区间。

## 原地写回

`update_surface_mesh_from_tet_dofs(...)` 会复用已经构造好的 `ObjMeshData` 和缓存的 `TetSurfaceMapping`：

1. 用当前 DOF 向量更新 surface 顶点位置
2. 重新计算法线

这样就不需要每帧重复做边界提取。

当前 scene bridge 走的就是这条路径：

```text
TetBody
  -> build_tet_surface_mapping(...)   (一次)
  -> tet_rest_to_surface_mesh(...)    (一次)
  -> update_surface_mesh_from_tet_dofs(...)  (每次 sync)
```

## 当前限制

这一层不会：

- 保留单独的高分辨率显示网格
- 把任意 display mesh 嵌入 tet mesh
- 从 surface mesh 反推 tet 数据

它默认 render surface 就是当前 tet topology 的 boundary surface。
