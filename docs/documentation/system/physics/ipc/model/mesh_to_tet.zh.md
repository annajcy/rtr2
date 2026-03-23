# `mesh_to_tet.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` 保存 surface mesh 到 tet 的入口接口。

## 当前职责

这个文件负责 `ObjMeshData -> TetGeometry / TetBody` 这一个方向。

公开 API 包括：

- `TetMeshingParams`
- `TetMeshingResult`
- `ftetwild_available()`
- `tetrahedralize_obj_mesh(...)`
- `obj_mesh_to_tet_geometry(...)`
- `obj_mesh_to_tet_body(...)`
- `obj_mesh_to_eigen_positions(...)`
- `obj_mesh_to_triangle_indices(...)`

这个 header 在 `src/rtr/` 内仍然保持 header-only；真正的 tetrahedralizer 由外部库提供，目前通过 `RTR_HAS_FTETWILD` 控制。

## 它做什么

当前转换链路是：

```text
ObjMeshData
  -> obj_mesh_to_eigen_positions()
  -> obj_mesh_to_triangle_indices()
  -> GEO::Mesh
  -> fTetWild tetrahedralization
  -> TetGeometry
  -> 可选地构造 TetBody
```

辅助函数会检查：

- vertices 非空
- indices 非空
- indices 必须按三角形组织
- 三角形索引不能越界

## fTetWild 边界

当 `RTR_HAS_FTETWILD=0` 时：

- `ftetwild_available()` 返回 `false`
- `tetrahedralize_obj_mesh(...)` 返回 `success=false`
- 便利接口会抛出带说明的异常

当 `RTR_HAS_FTETWILD=1` 时，这个 header 会：

- 构造 `GEO::Mesh` 表面
- 把 `TetMeshingParams` 映射到 `floatTetWild::Parameters`
- 把 fTetWild 输出矩阵回填成 `TetGeometry`

实现还会统一 tet 朝向；退化输出或越界输出会直接报错。

## 结果类型

`TetMeshingResult` 是非抛异常边界：

- 成功时 `success=true`，并填充 `geometry`
- 失败时 `success=false`，并填充 `error_message`

`obj_mesh_to_tet_geometry(...)` 和 `obj_mesh_to_tet_body(...)` 是给偏好异常路径的调用方准备的便利接口。

`obj_mesh_to_tet_body(...)` 返回的 body 仍然保留“先生成，再 author `fixed_vertices`”的用法，因此和现有 IPC 初始化流程兼容：

```text
ObjMeshData
  -> obj_mesh_to_tet_body(...)
  -> 设置 fixed_vertices / material
  -> add IPCTetComponent
```

## 当前限制

这一层不会尝试：

- 修复任意三角网格
- 保留原始 render mesh 拓扑
- 缓存 tetrahedralization 结果
- 隐藏运行时重 meshing 的成本

它本质上是一个 external tetrahedralization adapter，而不是完整的几何处理框架。
