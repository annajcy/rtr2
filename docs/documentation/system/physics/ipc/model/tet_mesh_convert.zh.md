# `tet_mesh_convert.hpp`

`src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp` 保存了当前 tet/mesh 转换层的实现。

## 当前边界

这个文件现在有两类职责：

- 正式路径：把 `TetGeometry` / `TetBody` / `IPCState::x` 转成可渲染的 `ObjMeshData`
- 辅助路径：把 `ObjMeshData` 拆成未来外部 tetrahedralizer 可消费的位置数组和三角数组

它当前**不做**：

- surface mesh 的 tetrahedralization
- `ObjMeshData -> TetGeometry`
- 任意三角网格的拓扑修复、封闭性修复或方向修复

未来的目标链路是：

```text
ObjMeshData
  -> mesh_positions_to_eigen()
  -> mesh_triangles()
  -> external tetrahedralizer
  -> TetGeometry
```

## `extract_tet_surface(...)`

表面提取依赖一个很直接的拓扑规则：

$$
\text{surface face} \iff \text{这个三角面只出现一次}
$$

每个 tet 会贡献 4 个三角面。实现里先对每个面顶点 id 排序，得到一个用于计数的 key，同时保留该面原来的局部有向顺序作为输出 winding。

最终结果保存在 `TetSurfaceResult` 中：

- `surface_indices`：边界三角面索引，仍然引用原始 tet 顶点号
- `surface_vertex_ids`：所有出现在边界上的唯一 tet 顶点号

之所以拆成这两部分，是因为渲染 surface 只会用到体网格顶点的一个子集。

## `tet_to_mesh(...)`

`tet_to_mesh(...)` 的作用是把 `TetSurfaceResult` 压缩成一个紧凑的 `ObjMeshData`。

### Step 1：建立紧凑的 surface 顶点映射

函数首先根据 `surface_vertex_ids` 构造一个映射：

$$
\text{old tet vertex id} \rightarrow \text{new surface vertex id}
$$

这样渲染网格里就不会保留内部顶点。

### Step 2：读取顶点位置

当前有两个入口：

- `tet_to_mesh(const Eigen::VectorXd& positions, ...)`
- `tet_to_mesh(const TetGeometry& geometry, ...)`

前者从全局 `3N` 自由度向量中读取当前顶点坐标：

$$
x_i =
\begin{bmatrix}
\mathbf{x}_{3i} \\
\mathbf{x}_{3i+1} \\
\mathbf{x}_{3i+2}
\end{bmatrix}
$$

后者则直接读取 `geometry.rest_positions`。

### Step 3：重写三角面索引

`surface_indices` 里还是 tet 全局顶点号，所以函数会把它们重写成紧凑的、以 0 为起点的 `ObjMeshData::indices`。

## 法线重计算

当位置和三角面索引都装配完成后，代码会根据几何重新计算顶点法线。对于一个三角形 `(i_0, i_1, i_2)`，面法线是：

$$
n_f = \mathrm{normalize}\left((p_1 - p_0) \times (p_2 - p_0)\right)
$$

每个三角面的法线会累加到相邻三个顶点，再对每个顶点做归一化。如果某个顶点最终累积出的法线长度为 0，当前实现会回退到 `(0, 1, 0)`。

## `update_mesh_positions(...)`

表面提取和顶点重映射只依赖 tet 拓扑，不依赖当前形变，所以可以缓存成 `TetSurfaceResult`。

`update_mesh_positions(...)` 就是复用这份缓存：

1. 用最新的 DOF 向量更新已经映射好的 surface 顶点位置
2. 重新计算法线

这比每一帧都重新跑一遍边界提取便宜得多。

## Mesh 到 Tet 的辅助函数

`mesh_positions_to_eigen(...)` 会把 `ObjMeshData` 中的顶点位置转成 `std::vector<Eigen::Vector3d>`。

`mesh_triangles(...)` 会把 flat 的 `indices` 数组重组为 `std::vector<std::array<uint32_t, 3>>`，并检查索引长度和范围是否合法。

这两个函数的职责刻意收得很窄。它们只是为未来的外部 tetrahedralizer 准备输入，并不会自己生成 `TetGeometry`。
