# IPC Core 总览

`core/` 目录保存的是面向求解器的全局状态，它不绑定某一个具体的 tet body。

当前内容只有：

- `src/rtr/system/physics/ipc/core/ipc_state.hpp`：未来 FEM/IPC 求解器使用的全局 `3N` 状态向量
- `src/rtr/system/physics/ipc/core/ipc_system.hpp`：系统级的装配和 stepping 封装，负责持有状态、body 列表和 Newton 回调

这一层刻意不持有：

- tet 拓扑
- 材料参数
- 边界条件元数据
- 渲染网格转换逻辑

这些内容都属于 `model/`。`core/` 的职责就是把未知量组织成求解器最容易消费的形式。
