# Mesh Tet Converter 总览

`src/rtr/system/physics/ipc/model/mesh_tet_converter/` 存放的是表面网格数据和四面体体网格数据之间的桥接层。

当前职责包括：

- `mesh_to_tet.hpp`：把 render-facing mesh 数据四面体化或转换成 tet body 输入
- `tet_to_mesh.hpp`：从 tet 状态中提取或更新可渲染表面网格

这层目录就是 IPC 模拟数据和场景可渲染数据之间的几何转换边界。
