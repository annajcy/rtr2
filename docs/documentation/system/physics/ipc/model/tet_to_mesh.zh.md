# `tet_to_mesh.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp` 是 IPC 运行时里负责 **tet volume → renderable surface mesh** 的核心桥接文件。

如果说 `mesh_to_tet.hpp` 负责把资产表面 mesh 送进求解域，那么 `tet_to_mesh.hpp` 就负责把求解域里的体网格状态重新投影回渲染层。

它补上的链路是：

```text
TetGeometry / TetBody / IPCState::x
  -> tet_to_mesh.hpp
  -> ObjMeshData
  -> DeformableMeshComponent
  -> GPU 动态网格
```

---

## 1. 为什么需要 `tet_to_mesh`

IPC/FEM 求解器关心的是体网格自由度，而 renderer 关心的是表面三角网格。

求解器里真正被更新的是：

$$
x \in \mathbb{R}^{3N}
$$

也就是所有体顶点拼接后的全局自由度向量。

但渲染系统需要的是：

- 顶点位置
- 顶点法线
- 三角面索引

换句话说，渲染层真正消费的是：

$$
\mathcal{S}_{render} = (V_{surf}, F_{surf})
$$

其中：

- $V_{surf}$ 是表面顶点
- $F_{surf}$ 是表面三角面

而求解器持有的是：

$$
\mathcal{T} = (V_{tet}, T)
$$

因此运行时必须回答三个问题：

1. 哪些 tet 面属于边界表面？
2. 边界表面用到了哪些 tet 顶点？
3. 当前 global DOF 向量中的哪一段对应这些顶点？

`tet_to_mesh.hpp` 正是对这三个问题的工程化回答。

---

## 2. 理论背景：tet boundary surface 是怎么定义的

### 2.1 边界面的拓扑定义

一个四面体单元有 4 个三角面。  
对于整个体网格来说：

- 如果某个三角面被两个相邻 tet 共享，它是内部面
- 如果某个三角面只出现一次，它是边界面

因此边界判定规则可以写成：

$$
\text{surface face} \iff \text{该三角面在所有 tet 面中恰好出现一次}
$$

这是一个纯拓扑定义，不依赖当前形变。

这点非常重要，因为它意味着：

> 只要 tet topology 不变，表面提取结果就可以缓存。

### 2.2 为什么 surface vertex 要单独压缩

体网格中很多顶点在内部，不会出现在渲染表面上。  
如果直接把所有 tet 顶点都塞给 renderer：

- 会带来无意义的数据上传
- 索引也会保留很多内部顶点“空洞”
- 动态写回成本更高

因此需要构造一个紧凑映射：

$$
\text{tet vertex id} \rightarrow \text{surface-local vertex id}
$$

这正是 `TetSurfaceMapping` 的意义。

### 2.3 法线为什么要重算

求解器更新的是顶点位置，不会自动维护渲染法线。  
而 deformable mesh 的形状在每个 fixed tick 都会变化，因此法线也必须跟着更新。

如果一个三角面为 $(i_0, i_1, i_2)$，其面法线可以写成：

$$
n_f = \mathrm{normalize}((p_1 - p_0) \times (p_2 - p_0))
$$

然后把面法线累加到三个顶点，最后再做顶点归一化。这就是当前实现里的 per-frame normal recompute 逻辑。

---

## 3. 系统设计：为什么单独拆成 `tet_to_mesh.hpp`

### 3.1 它是运行时高频路径

和 `mesh_to_tet.hpp` 相比，`tet_to_mesh.hpp` 有完全不同的生命周期：

- `build_tet_surface_mapping(...)`：初始化时做一次
- `tet_rest_to_surface_mesh(...)`：初始化时做一次
- `update_surface_mesh_from_tet_dofs(...)`：每次 fixed tick 都可能调用

因此它不是“导入工具”，而是**场景桥接和渲染写回的运行时路径**。

### 3.2 为什么不让 `DeformableMeshComponent` 直接理解 tet

`DeformableMeshComponent` 的职责应该保持渲染侧：

- 持有 deformable mesh 资源句柄
- 提供 mesh view 给 GPU
- 接受已经准备好的表面 positions/normals

而不应该直接知道：

- tet topology
- global DOF offset
- 哪些面属于体网格边界

这些求解域语义由 `tet_to_mesh.hpp` 和 `IPCTetComponent` 更适合承担。

### 3.3 为什么 `TetSurfaceMapping` 要缓存

边界提取本身虽然不算极重，但它仍然是一个遍历全部 tet 的操作：

$$
O(|T|)
$$

如果每一帧都重新做：

- 多 body 场景里会重复浪费 CPU
- scene bridge 会承担没必要的拓扑工作

而一旦 `TetSurfaceMapping` 被缓存，后续每帧只需要：

- 读取表面顶点对应位置
- 重算法线

这使得运行时路径更轻、更清楚。

---

## 4. 公开 API 的设计意图

### 4.1 `TetSurfaceMapping`

当前结构是：

- `surface_indices`
- `surface_vertex_ids`

这两个数组的职责不同：

`surface_vertex_ids`

- 保存所有参与边界的 tet 顶点 id
- 这些 id 仍然是 body-local tet 顶点编号

`surface_indices`

- 保存边界三角面的顶点索引序列
- 这里的顶点号仍然先指向 tet 顶点 id，而不是紧凑 surface-local id

这个设计看起来有些“中间态”，但它很有价值：

- 一方面保留了和 tet topology 的直接联系
- 另一方面又足够表达紧凑表面构造所需的信息

### 4.2 `build_tet_surface_mapping(...)`

这个函数只依赖拓扑，不依赖当前形变。  
因此它既可以接受：

- `TetGeometry`
- 也可以接受 `TetBody`

它的职责是：

> 从体网格中提取“边界面 + 边界顶点集合”。

### 4.3 `tet_rest_to_surface_mesh(...)`

这是初始化路径。

它回答的问题是：

> 如果当前体网格还没开始形变，初始渲染 surface 应该长什么样？

因此它直接从：

- `geometry.rest_positions`

读取顶点位置，构造第一份 `ObjMeshData`。

### 4.4 `tet_dofs_to_surface_mesh(...)`

这是“临时构造一份表面 mesh”的通用接口。  
相比 `update_surface_mesh_from_tet_dofs(...)`，它会重新创建一个新的 `ObjMeshData`。

因此更适合：

- 测试
- 调试
- 单次导出

而不是高频 scene write-back。

### 4.5 `update_surface_mesh_from_tet_dofs(...)`

这是运行时真正重要的接口。

它解决的问题是：

> 已经有一份 mesh cache 的情况下，如何从最新 DOF 向量原地更新它？

这正是 scene bridge 需要的行为。

---

## 5. 代码实现讲解

下面按实现结构说明关键算法。

### 5.1 `Face` / `FaceHash` / `FaceData`

为了统计“一个三角面出现了几次”，实现定义了：

- `Face`：排序后的三角面 key
- `FaceHash`：unordered_map 的 hash
- `FaceData`：出现次数 + 一份有向顶点顺序

这里有个关键设计点：

#### 为什么 key 要排序

两个相邻 tet 共享同一个几何三角面时，局部顶点顺序通常会不同。  
如果直接拿 `(a,b,c)` 做 key，那么：

- `(1,2,3)` 和 `(2,1,3)` 会被误判为两个不同面

所以必须先排序，得到一个“无向拓扑 key”。

#### 为什么还要保存 `oriented_vertices`

因为渲染输出仍然需要有向三角面顺序。  
排序只是为了计数，不能直接拿排序后的顶点顺序作为输出 winding。

因此 `FaceData` 同时保存：

- `count`
- `oriented_vertices`

这是一个典型的“计数 key 和输出表示分离”的实现。

### 5.2 `build_tet_surface_mapping(...)`

实现步骤如下：

1. 遍历所有 tet
2. 对每个 tet 加入 4 个三角面
3. 用排序后的 `Face` 做哈希 key
4. 统计每个面出现次数
5. 只保留 `count == 1` 的面
6. 收集这些面的顶点，形成 `surface_vertex_ids`

tet 的四个面当前写成：

```text
(0,1,3)
(1,2,3)
(2,0,3)
(0,2,1)
```

这里默认输入 tet 朝向已经被规范化，因此局部面顺序也能保持一致的表面绕序语义。

得到 boundary faces 后，代码还会：

- 用 `std::vector<bool> is_surface_vertex` 去重
- 最后把 `surface_vertex_ids` 排序

排序的好处是：

- 输出更稳定
- 测试更稳定
- 后续 surface-local 压缩映射也更直观

### 5.3 `build_surface_mesh_from_mapping(...)`

这个模板函数是整个文件最核心的“共用装配器”。

它把“如何读取一个顶点位置”抽象成 `PositionReader`，然后统一完成：

1. 根据 `surface_vertex_ids` 创建紧凑 surface 顶点数组
2. 建立 `old_to_new` 映射
3. 重写 `surface_indices`
4. 重新计算法线

这一抽象的价值是：

- `tet_rest_to_surface_mesh(...)` 和 `tet_dofs_to_surface_mesh(...)` 共享相同的索引重写逻辑
- 只在“位置从哪里来”这件事上不同

从设计上，这是一种很典型的“把 topology 装配和 geometry 读取解耦”的写法。

### 5.4 `validate_surface_mapping(...)`

这里有两组重载：

- `validate_surface_mapping(surface, vertex_count)`
- `validate_surface_mapping(surface, vertex_count, vertex_offset)`

它们分别服务于：

- 直接读取 body-local 几何
- 从 global DOF 向量里读取多 body 的局部片段

检查内容包括：

- `surface_vertex_ids` 是否越界
- `surface_indices` 是否越界

注意这里检查的 `surface_indices` 仍然是 tet 顶点 id，而不是已经压缩后的渲染索引。

### 5.5 `tet_rest_to_surface_mesh(...)`

这个函数只做两件事：

1. 校验 `surface` 是否和 `geometry` 一致
2. 把 `geometry.rest_positions[vertex_id]` 作为位置源传给 `build_surface_mesh_from_mapping(...)`

它是最直接的初始化路径，因此常被 `IPCTetComponent` 构造阶段使用。

### 5.6 `tet_dofs_to_surface_mesh(...)`

这个函数的理论输入是：

$$
x =
\begin{bmatrix}
x_0^x & x_0^y & x_0^z & x_1^x & x_1^y & x_1^z & \cdots
\end{bmatrix}^T
$$

也就是标准的 `3N` 向量。

给定顶点 id $i$，其位置在向量中的基址为：

$$
3i
$$

这也是 `read_position_from_dofs(...)` 的实现依据。

当一个 body 处于 global state 的某个偏移位置时，还需要：

$$
i_{global} = i_{local} + \text{vertex\_offset}
$$

因此 `vertex_offset` 是 scene bridge 支持多 body 的关键参数。

### 5.7 `update_surface_mesh_from_tet_dofs(...)`

这是整个文件最关键的运行时函数。

它不会重建 mesh，而是直接：

1. 检查 `mesh.vertices.size()` 是否等于 `surface.surface_vertex_ids.size()`
2. 检查 `mesh.indices.size()` 是否等于 `surface.surface_indices.size()`
3. 用新的 DOF 更新每个 surface 顶点位置
4. 调 `recompute_mesh_normals(...)`

这条路径比“每帧重新构造一个 ObjMeshData”更高效，因为：

- 复用已有 CPU mesh 容器
- 索引不变
- 只更新位置和法线

### 5.8 `recompute_mesh_normals(...)`

法线重算分两步：

#### Step 1：面法线累加

对每个三角形：

$$
n_f = \mathrm{normalize}((p_1 - p_0) \times (p_2 - p_0))
$$

然后把这个面法线累加到三个顶点。

#### Step 2：顶点归一化

对每个顶点，把累积法线归一化。

如果某个顶点累积法线长度为 0，则回退到：

$$
(0,1,0)
$$

这是一种工程上简单稳定的兜底策略，可以避免出现 NaN 法线。

---

## 6. 它在当前系统中的位置

### 6.1 和 `IPCTetComponent` 的关系

当前 `IPCTetComponent` 的典型初始化流程是：

```text
TetBody
  -> build_tet_surface_mapping(...)
  -> tet_rest_to_surface_mesh(...)
  -> mesh_cache
```

这意味着：

- `surface_cache` 存的是拓扑映射
- `mesh_cache` 存的是可复用的表面 CPU mesh

### 6.2 和 `sync_ipc_to_scene(...)` 的关系

scene bridge 在每次 fixed tick 后做：

```text
IPCState::x
  -> body.info.dof_offset / 3
  -> update_surface_mesh_from_tet_dofs(...)
  -> DeformableMeshComponent::apply_deformed_surface(...)
```

因此 `tet_to_mesh.hpp` 实际上是：

- IPC 运行时
- framework scene bridge
- render mesh resource

三者之间的“数据翻译器”。

### 6.3 为什么 `vertex_offset` 很重要

单 body 情况下，`surface_vertex_ids` 可以直接索引本地顶点。  
但一旦 `IPCState::x` 把多个 body 拼接起来，就必须知道：

- 当前 body 的顶点在全局向量里从哪里开始

这就是：

$$
\text{vertex\_offset} = \frac{\text{body.info.dof\_offset}}{3}
$$

的意义。

没有这个偏移，多 body 场景里就会把一个 body 的 surface 顶点错误地读到另一个 body 的 DOF 区间。

---

## 7. 当前设计的边界与代价

### 7.1 当前默认 render mesh 就是 tet boundary

这是一个非常重要的系统假设。

当前实现默认：

> 渲染用的 deformable mesh 就是从 tet boundary 提取出来的表面网格。

这意味着它**不支持**：

- 原始高分辨率显示网格保持不变
- 仿真用粗 tet，显示用另一个独立 surface
- embedded surface / skinning-style 映射

### 7.2 这个假设为什么在当前阶段合理

因为它能显著降低系统复杂度：

- 不需要额外的 surface embedding 数据结构
- 不需要 barycentric/shape function 映射
- `sync_ipc_to_scene(...)` 的写回路径非常直接

在当前项目阶段，这是一种很合适的最小闭环设计。

### 7.3 代价是什么

代价是显示拓扑受 tet boundary 拓扑约束。

一旦将来需要：

- 高精度显示
- 低精度仿真
- 独立的碰撞代理表面

就要在这个层之上再增加一层映射系统，而不是继续扩展当前 `TetSurfaceMapping` 的职责。

---

## 8. 总结

`tet_to_mesh.hpp` 的核心价值，是把“求解器内部的体网格自由度”翻译成“渲染系统能消费的表面 mesh”，并且把高频运行时路径压缩到一个简单清晰的模式：

```text
初始化：
  build_tet_surface_mapping(...)
  + tet_rest_to_surface_mesh(...)

每帧写回：
  update_surface_mesh_from_tet_dofs(...)
```

它补齐了当前 deformable runtime 最关键的一步：

- 求解器更新的是体网格
- 但屏幕上显示的是表面网格

而这份 header 就是两者之间稳定、可缓存、可测试的转换层。
