# `mesh_to_tet.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` 是 IPC 几何适配层里负责 **surface mesh → volumetric tet mesh** 的入口文件。

它的角色不是“一个通用几何处理库”，而是一个**面向当前 IPC/FEM 运行时的数据桥接器**：

- 输入侧接收引擎内部通用的 `rtr::utils::ObjMeshData`
- 中间把表面三角网格转换成外部 tetrahedralizer 可消费的格式
- 输出侧产出 `TetGeometry` 或 `TetBody`

从架构上看，它正好补上了这条链路前半段：

```text
ObjMeshData
  -> mesh_to_tet.hpp
  -> TetGeometry / TetBody
  -> IPCTetComponent
  -> IPCSystem
```

---

## 1. 为什么需要 `mesh_to_tet`

在 IPC/FEM 系统里，求解器真正需要的是**体网格**，不是单纯的三角表面。

对于一个四面体体网格，弹性能、惯性能、质量矩阵、Dirichlet 约束、全局自由度装配，全部都依赖下面这组数据：

- 顶点参考位置 `rest_positions`
- 四面体单元连接关系 `tets`
- 可选的 body 级材料参数

换句话说，求解器需要的是：

$$
\mathcal{T} = (V, T)
$$

其中：

- $V$ 是三维顶点集合
- $T$ 是四面体单元集合

而美术资源、OBJ loader、渲染管线里最常见的却是表面三角网格：

$$
\mathcal{S} = (V_s, F_s)
$$

其中：

- $V_s$ 是表面顶点
- $F_s$ 是三角面集合

从 $\mathcal{S}$ 到 $\mathcal{T}$ 不是简单格式转换，而是一个真正的**体网格生成问题**：

- 要在封闭表面内部填充四面体
- 要保证单元索引合法
- 要尽量避免退化和倒置单元
- 要把结果整理成求解器能直接消费的内部表示

因此 `mesh_to_tet.hpp` 的本质职责是：

> 把“渲染/资产层的表面 mesh”翻译成“IPC/FEM 层的体网格数据”。

---

## 2. 理论背景：为什么 tetrahedralization 不能自己随便写

### 2.1 表面 mesh 不是体网格

OBJ / glTF 之类的资产格式通常只描述**边界表面**。  
即使一个模型看起来是“封闭的实体”，文件里通常也只有外壳三角面。

这意味着，输入文件只告诉我们：

- 外边界长什么样

但没有直接告诉我们：

- 内部该如何剖分成四面体

### 2.2 体网格质量会直接影响求解稳定性

对 FEM/IPC 来说，四面体质量非常重要。  
过于扁平、接近共面的单元会导致：

- 参考构型矩阵接近奇异
- 形变梯度计算不稳定
- 能量/Hessian 条件数恶化
- Newton 求解更难收敛

在一个 tet 单元里，如果把参考构型矩阵写成：

$$
D_m =
\begin{bmatrix}
x_1 - x_0 & x_2 - x_0 & x_3 - x_0
\end{bmatrix}
$$

那么：

- $\det(D_m)$ 的绝对值和体积成正比
- $\det(D_m) \approx 0$ 表示单元接近退化
- $\det(D_m) < 0$ 表示局部顶点顺序导致朝向反了

这也是为什么当前实现会在把 fTetWild 输出回填到 `TetGeometry` 时显式检查：

- tet 索引是否越界
- determinant 是否过小
- 如果 determinant 为负，则交换两个顶点统一正方向

### 2.3 为什么这里选择 external tetrahedralizer

体网格生成本身是一个成熟但复杂的几何处理子领域。  
当前仓库不打算在 `src/rtr/` 内部手写 tetrahedralization 算法，而是采用：

- `src/rtr/` 保持 header-only adapter
- 真正的 volume meshing 由外部库完成

当前后端是 fTetWild，它擅长：

- 从三角表面生成体四面体网格
- 对一般工程输入比“自己手写”更稳

因此 `mesh_to_tet.hpp` 的设计哲学不是“重复造轮子”，而是：

> 在引擎数据结构与外部 tetrahedralizer 之间建立一个最薄、可控、可测试的边界层。

---

## 3. 系统设计：为什么单独拆成 `mesh_to_tet.hpp`

这次重构之后，`mesh_tet_converter/` 目录被明确拆成两个方向：

- `mesh_to_tet.hpp`：表面 mesh 到体网格
- `tet_to_mesh.hpp`：体网格到渲染 surface mesh

这样拆的核心原因是**方向不同，语义完全不同**。

### 3.1 输入/输出对象不同

`mesh_to_tet.hpp`：

- 输入：`ObjMeshData`
- 输出：`TetGeometry` / `TetBody`

`tet_to_mesh.hpp`：

- 输入：`TetGeometry` / `TetBody` / `IPCState::x`
- 输出：`ObjMeshData`

### 3.2 生命周期不同

`mesh_to_tet` 通常是：

- 初始化时做一次
- 资源导入时做一次
- 用户显式重建时做一次

它是一个**昂贵的初始化型操作**。

而 `tet_to_mesh` 是：

- scene bridge 初始化时做一次
- 每次 fixed tick write-back 时重复做轻量更新

它是一个**运行时高频路径**。

### 3.3 错误模型不同

`mesh_to_tet` 需要面对：

- 输入 surface mesh 非法
- tetrahedralizer 不可用
- 外部库失败
- 输出单元退化

所以它提供了两层接口：

- 非抛异常的 `TetMeshingResult`
- 抛异常的 convenience wrappers

而 `tet_to_mesh` 更多是内部拓扑/缓存一致性问题，错误模型更偏“程序员错误”或“状态不同步”。

### 3.4 为什么保持 header-only

当前仓库对 `src/rtr/` 的约束是继续保持 header-only。  
因此这里采用的策略是：

- 在 `src/rtr/` 内部只写 `inline` adapter
- 外部依赖通过 CMake 链接编译后的第三方库

这使得当前文件既满足工程边界，又不会把 volume meshing 的复杂实现揉进引擎内部源码结构。

---

## 4. 公开 API 的设计意图

### 4.1 `TetMeshingParams`

`TetMeshingParams` 是一层“引擎语义参数”，而不是把 fTetWild 的全部配置直接暴露给上层。

目前字段有：

- `ideal_edge_length_rel`
- `eps_rel`
- `max_its`
- `skip_simplify`
- `quiet`

设计意图是：

- 给 demo / editor / future importer 一个足够小的控制面
- 避免把第三方参数类型泄漏到更多业务代码里

其中最重要的是：

`ideal_edge_length_rel`

它控制目标单元尺度。一般来说：

- 值越小，网格越细，tet 数越多
- 值越大，网格越粗，tet 数越少

因此它直接控制：

- tetrahedralization 时间
- 后续 IPC/FEM 求解规模
- 视觉细节与体网格成本之间的平衡

### 4.2 `TetMeshingResult`

`TetMeshingResult` 提供了一个显式的失败边界：

- `success`
- `geometry`
- `error_message`

为什么要保留这个结果结构，而不是所有地方都抛异常？

因为 `mesh_to_tet` 同时服务两类调用方：

- 工程型路径：希望稳定失败并给 UI/日志一个错误消息
- 便利路径：希望失败时直接抛异常

因此设计成：

- `tetrahedralize_obj_mesh(...)`：非异常边界
- `obj_mesh_to_tet_geometry(...)` / `obj_mesh_to_tet_body(...)`：异常边界

### 4.3 `obj_mesh_to_tet_body(...)`

这个接口的价值不只是“少写几行代码”，更关键的是它把运行时真正需要的 authoring 对象直接构造出来。

当前返回的 `TetBody` 仍然保留一个重要约束：

> 它不会替你 author `fixed_vertices`。

这点是有意为之。因为在当前系统里：

- meshing 决定几何和 topology
- 约束 authoring 决定边界条件

这两个阶段应当分开。

所以典型用法是：

```text
ObjMeshData
  -> obj_mesh_to_tet_body(...)
  -> 选择 fixed_vertices
  -> add IPCTetComponent
```

---

## 5. 代码实现讲解

下面按实现顺序说明这份 header 的关键部分。

### 5.1 `obj_mesh_to_eigen_positions(...)`

这个函数做的事情很简单，但它定义了一个重要的**类型边界**：

- 引擎资产层里，顶点位置是 `pbpt::math::Vec3`，通常走 `float`
- tetrahedralization 和 FEM 几何计算更适合 `double`

因此这里把每个顶点位置转成：

$$
\mathbf{x}_i \in \mathbb{R}^3
$$

对应的 `Eigen::Vector3d`。

这个步骤的目的不是做几何处理，而是：

- 进入数值几何 / tetrahedralizer 更常见的 double 精度表示
- 为后续 `GEO::Mesh` 构造准备输入

### 5.2 `obj_mesh_to_triangle_indices(...)`

这个函数把 `ObjMeshData` 里 flat 的 `indices`：

```text
i0, i1, i2, i3, i4, i5, ...
```

重组为：

```text
{i0, i1, i2}, {i3, i4, i5}, ...
```

它做了两个关键检查：

1. `indices.size() % 3 == 0`
2. 每个索引都必须落在 `vertices.size()` 范围内

这一步虽然简单，但它防止了最典型的未定义行为来源：

- 非三角索引输入
- 越界索引访问

### 5.3 `validate_obj_mesh(...)`

`validate_obj_mesh(...)` 是更高层的入口校验：

- 顶点不能为空
- 索引不能为空
- 三角拓扑与范围合法性委托给 `obj_mesh_to_triangle_indices(...)`

它的设计是“把输入错误尽量压在最靠近入口的地方暴露”，而不是等到外部 tetrahedralizer 再崩。

### 5.4 `initialize_ftetwild_runtime()`

这部分只在 `RTR_HAS_FTETWILD` 开启时存在。

Geogram/fTetWild 依赖底层运行时初始化，所以这里用：

- `std::once_flag`
- `std::call_once`

保证 `GEO::initialize()` 只做一次。

这是一个典型的 adapter 设计细节：

- 上层 API 不关心底层库初始化顺序
- 但 header 自己要把这件事封在边界里

### 5.5 `build_geo_surface_mesh(...)`

这是从引擎数据结构到外部库数据结构的关键桥接点。

内部过程是：

```text
ObjMeshData
  -> std::vector<Eigen::Vector3d>
  -> std::vector<std::array<uint32_t, 3>>
  -> std::vector<floatTetWild::Vector3>
  -> std::vector<floatTetWild::Vector3i>
  -> GEO::Mesh
```

这里最重要的系统设计意义是：

- `ObjMeshData` 是引擎统一 mesh 表示
- `GEO::Mesh` 是 tetrahedralizer 所需表示
- `mesh_to_tet.hpp` 明确负责这层翻译

这样上层业务就不用知道：

- Geogram 的 mesh 构造细节
- fTetWild 的输入容器类型

### 5.6 `tetrahedralize_obj_mesh(...)`

这是整个文件的主入口。

它按顺序做：

1. 校验输入 mesh
2. 初始化 fTetWild 运行时
3. 构造 `GEO::Mesh`
4. 把 `TetMeshingParams` 映射到 `floatTetWild::Parameters`
5. 调用 `floatTetWild::tetrahedralization(...)`
6. 把输出矩阵整理成 `TetGeometry`
7. 捕获异常并转成 `TetMeshingResult`

这里有几个重要设计选择。

#### 选择 A：把第三方失败变成引擎自己的失败模型

fTetWild 返回的是：

- 返回码
- 输出矩阵
- 以及可能抛出的底层异常

而引擎对外希望看到的是统一的：

- `success`
- `error_message`

所以这里承担了“错误语义翻译层”的职责。

#### 选择 B：只暴露必要参数

当前实现没有把 fTetWild 的所有旋钮暴露出来，而是只暴露：

- 网格尺度
- 容差
- 迭代上限
- 是否跳过简化
- 是否静默

这保证 API 更稳定，也更适合当前项目阶段。

### 5.7 `build_tet_geometry_from_matrices(...)`

这是从外部库输出回到引擎内部表示的关键步骤。

fTetWild 返回的是矩阵：

- 顶点矩阵 `vertices`，形状为 `N x 3`
- 四面体矩阵 `tets`，形状为 `M x 4`

而引擎内部要的是：

- `std::vector<Eigen::Vector3d> rest_positions`
- `std::vector<std::array<std::size_t, 4>> tets`

回填时，代码会做三类检查。

#### 检查 1：矩阵形状

- 顶点必须 3 列
- tet 必须 4 列

这对应最基本的几何语义约束。

#### 检查 2：索引范围

tet 里每个顶点索引必须满足：

$$
0 \le i < |V|
$$

否则说明外部输出已经不合法。

#### 检查 3：退化和朝向

对每个 tet 构造：

$$
D_m =
\begin{bmatrix}
x_1 - x_0 & x_2 - x_0 & x_3 - x_0
\end{bmatrix}
$$

然后检查：

- $\lvert \det(D_m) \rvert$ 是否过小
- 如果 $\det(D_m) < 0$，交换两个顶点统一正方向

这一点非常关键，因为当前 `TetGeometry` / `TetBody` 的下游默认假定：

- 单元不是退化的
- 单元方向已经规范化

也就是说，这个函数承担了“把外部 tetrahedralizer 输出整理成内部可求解几何”的最后清洗步骤。

### 5.8 `obj_mesh_to_tet_geometry(...)` 和 `obj_mesh_to_tet_body(...)`

这两个函数的实现都很薄，但它们体现了系统接口设计：

- `obj_mesh_to_tet_geometry(...)`：适合几何层或测试直接消费
- `obj_mesh_to_tet_body(...)`：适合 runtime authoring 直接消费

`obj_mesh_to_tet_body(...)` 还会补：

- `body.material`
- `body.info.type`
- `body.info.vertex_count`

因此它产出的对象已经是一个合理的 scene/runtime authoring 基础单元。

---

## 6. 参数如何影响系统行为

### 6.1 `ideal_edge_length_rel`

这是最直接影响 meshing 分辨率的参数。

经验上：

- 更小：tet 更细，初始化更慢，求解更重
- 更大：tet 更粗，初始化更快，求解更轻

它同时影响两个阶段的成本：

1. **导入/初始化成本**：fTetWild 要处理更多单元
2. **运行时成本**：IPC/FEM 的自由度更多，Newton 求解更重

### 6.2 `eps_rel`

这是一个相对容差参数，用来控制 meshing 过程里的几何容忍度。  
它不是“视觉质量滑条”，更像数值稳定性和保真度之间的一个几何参数。

### 6.3 `max_its`

它限制底层 meshing 迭代上限。  
对工程上常见的含义来说：

- 更高：可能得到更稳定或更细致的结果，但初始化更慢
- 更低：更快，但质量或收敛结果可能更差

### 6.4 `skip_simplify`

这个参数不是“跳过整个 tetrahedralization”，而是跳过某部分简化路径。  
它本质上是在 meshing 质量与速度之间做取舍。

---

## 7. 这一层和其他系统的关系

### 7.1 和 `TetBody` 的关系

`mesh_to_tet.hpp` 产出的不是最终求解结果，而是求解前 authoring 阶段的输入。

`TetBody` 后续还会参与：

- `precompute()`
- mass / inverse mass / rest shape data
- `fixed_vertices`
- body-level material dispatch

所以 `mesh_to_tet` 只负责**把几何准备好**。

### 7.2 和 `IPCTetComponent` 的关系

当前 `IPCTetComponent` 仍然以 `TetBody` 为 scene 侧 authoring 输入。  
因此 `mesh_to_tet.hpp` 是 scene 初始化或导入逻辑的前置步骤，而不是 `IPCTetComponent` 内部每帧调用的运行时逻辑。

这也是为什么当前设计更接近：

```text
surface asset
  -> mesh_to_tet
  -> TetBody
  -> IPCTetComponent
```

而不是：

```text
IPCTetComponent
  -> 每帧重新从 render mesh tetrahedralize
```

后者从性能和语义上都不合适。

### 7.3 和 `tet_to_mesh.hpp` 的关系

这两个文件在系统中的位置刚好相反：

- `mesh_to_tet.hpp`：进入求解域
- `tet_to_mesh.hpp`：离开求解域

前者把 surface asset 转成 volumetric simulation data。  
后者把 volumetric simulation state 转回 surface render data。

这种成对拆分的好处是：

- API 方向明确
- 生命周期明确
- 调试时可以清楚地区分“导入问题”与“写回问题”

---

## 8. 当前限制与后续扩展方向

当前实现刻意保持克制，它没有试图解决所有几何问题。

### 8.1 当前不做的事

- 任意三角网格修复
- watertightness 自动修补
- 输入/输出 surface 拓扑保持一致
- meshing 结果缓存
- 高分辨率显示网格与低分辨率仿真网格分离

### 8.2 后续可能的演化方向

如果未来要继续扩展，比较自然的方向包括：

1. 增加离线缓存  
   例如把 tetrahedralization 结果写入自定义 tet 资产格式，避免每次启动重算

2. 增加显示网格嵌入  
   即仿真用粗 tet，显示仍保留原始高分辨率 surface

3. 增加更丰富的 meshing authoring 配置  
   例如不同模型类型的默认参数、导入策略、失败回退策略

---

## 9. 总结

`mesh_to_tet.hpp` 的意义不在于“它调用了 fTetWild”，而在于它把一条原本跨越多个系统边界的链路收敛成了一个可控接口：

```text
ObjMeshData
  -> 校验
  -> 外部 tetrahedralizer
  -> TetGeometry / TetBody
  -> IPC authoring
```

它把：

- 资产层的 surface mesh
- 外部 tetrahedralizer
- IPC/FEM 的内部体网格表示

三者连接起来，并在边界上补齐了：

- 参数映射
- 输入校验
- 错误翻译
- 输出清洗

因此它是当前 deformable runtime 能从“程序生成的 block”走向“任意导入 surface mesh”的关键一步。
