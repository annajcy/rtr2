# IPC Model 总览

`model/` 目录保存的是 IPC 侧的几何和 body 元数据层。

当前文件包括：

- `src/rtr/system/physics/ipc/model/ipc_body.hpp`：body 类型和 body 到全局自由度的映射元数据
- `src/rtr/system/physics/ipc/model/tet_body.hpp`：四面体几何、参考构型预计算、body 级材料参数和 block 生成工具
- `src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp`：tet surface 提取和 tet 到渲染网格的转换
- `src/rtr/system/physics/ipc/model/obstacle_body.hpp`：占位用的 obstacle body 类型

这一层回答的问题是：

- 一个可形变 body 的参考构型长什么样
- 一个 body 如何映射到全局 `IPCState`
- tet 几何如何变成可渲染的表面网格

它目前还不负责能量、梯度、Hessian 或 line search，这些属于未来的 `energy/` 和 `solver/` 层。
