# Tet To Mesh

`src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp` 是 IPC model 层里把 tet 状态写回可渲染表面网格的桥接文件。

它负责根据 tet 拓扑和自由度数据生成或更新渲染代码可消费的表面 mesh 数据。

更完整的说明可以继续参考旧页面 [`../tet_to_mesh.zh.md`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/docs/documentation/system/physics/ipc/model/tet_to_mesh.zh.md)。
