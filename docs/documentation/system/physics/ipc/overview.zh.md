# IPC 总览

`src/rtr/system/physics/ipc/` 是未来可形变物体管线的数据模型层。它和已经接入运行时的 `src/rtr/system/physics/rigid_body/` 刚体路径刻意分开。

当前 IPC 目录按职责划分为：

- `core/`：面向求解器的全局状态容器
- `model/`：body 元数据、tet 几何以及 tet/mesh 转换辅助
- `energy/`：未来的弹性能、惯性能、重力、接触能入口
- `solver/`：未来的 Newton、line search 和系统装配入口

当前已经实现的是“进入 FEM/IPC 之前必须先存在的数据层”：

- 全局 `3N` 状态向量 `IPCState`
- body 到全局自由度的映射元数据 `IPCBodyInfo`
- 四面体参考构型与预计算 `TetGeometry`、`TetBody`
- surface 到 tet 的 meshing 入口 `mesh_tet_converter/mesh_to_tet.hpp`
- tet 到渲染网格的写回工具 `mesh_tet_converter/tet_to_mesh.hpp`

当前还没有实现：

- tet 弹性能计算
- 全局系统装配
- Newton / line search 求解循环
- contact barrier / CCD
- 通过 `step_scene_physics(...)` 和 `ipc_system.step(dt)` 接入的 deformable runtime

## 模块关系

当前设计的目标数据流是：

```text
TetGeometry / TetBody
    -> 预计算参考构型数据
    -> 装配进 IPCState 全局自由度
    -> 未来的 energy / solver 管线
    -> tet_rest_to_surface_mesh(...) / update_surface_mesh_from_tet_dofs(...)
    -> 供渲染使用的 ObjMeshData
```

`model/` 负责几何和映射元数据，`core/` 负责求解器直接操作的全局向量，未来 `energy/` 和 `solver/` 会同时消费这两层数据。

## 当前边界

当前 `ipc/` 子树的范围刻意收得比较窄：

- 已支持：节点自由度存储、tet 参考构型、顶点 lumped mass、tet surface 导出，以及在 `RTR_HAS_FTETWILD` 开启时的 `ObjMeshData -> TetGeometry` / `TetBody`
- 还不支持：任意拓扑修复、meshing 结果缓存，以及“显示网格独立于 tet boundary”的嵌入路径

具体数据布局和算法说明放在本目录下各个文件对应的页面中。
