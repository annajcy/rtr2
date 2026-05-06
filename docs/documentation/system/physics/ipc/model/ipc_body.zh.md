# `ipc_body.hpp`

`src/rtr/system/physics/ipc/model/ipc_body.hpp` 定义了 IPC 侧 body 共享的最小元数据。

## `IPCBodyType`

`IPCBodyType` 用来区分求解器当前面对的是哪一类 body：

- `Tet`
- `Shell`
- `Obstacle`

当前真正有实体结构的是 `Tet` 和占位的 `ObstacleBody`，但这个枚举已经为后续混合 body 系统留好了位置。

## `IPCBodyInfo`

`IPCBodyInfo` 是“单个 body”和“未来全局求解向量”之间的桥：

- `type`：body 的具体类别
- `dof_offset`：在 `IPCState` 里的起始全局 DOF 下标
- `vertex_count`：这个 body 拥有的顶点数
- `enabled`：当前是否参与装配和求解

其中最关键的是 `dof_offset`。因为全局 IPC 状态采用 vertex-major 布局，而且每个顶点贡献 3 个平移自由度，所以 `dof_offset` 指向的是该 body 在全局向量中的首个自由度。

如果一个 body 的顶点起始偏移是 `k`，那么：

$$
\text{dof\_offset} = 3k
$$

这个 body 在全局状态里的连续区间长度就是 `3 * vertex_count`。

## 为什么这份元数据保持得很小

`IPCBodyInfo` 刻意不重复保存几何、材料或质量数据。它只解决三件事：

- 标识 body 类型
- 描述局部顶点如何映射到全局向量
- 支持装配阶段按 body 开关启停

这样几何和材料仍然留在 `TetBody` 里，而求解器仍然能用一份很薄的元数据完成局部到全局的映射。
