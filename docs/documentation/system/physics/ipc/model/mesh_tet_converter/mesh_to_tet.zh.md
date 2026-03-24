# Mesh To Tet

`src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` 是 IPC model 层里把表面 mesh 转成四面体体网格输入的入口。

它负责把面向渲染的 mesh 输入转换成可进入 IPC 流程的 tet 模拟输入，通常用于构建 `TetGeometry` 和 `TetBody`。

更完整的算法说明和设计背景可以继续参考旧页面 [`../mesh_to_tet.zh.md`](/Users/jinceyang/Desktop/codebase/graphics/rtr2/docs/documentation/system/physics/ipc/model/mesh_to_tet.zh.md)。
