# `tet_body.hpp`

`src/rtr/system/physics/ipc/model/tet_body.hpp` 定义了 IPC 子树里使用的四面体体网格模型。

## 两层结构：`TetGeometry` 和 `TetBody`

这个文件刻意把一个 tet 对象拆成两层：

- `TetGeometry`：参考构型几何和按单元预计算的 rest-shape 数据
- `TetBody`：面向物理的包装层，补上 body 元数据、材料参数、边界条件和顶点质量

这样做的目的，是让“纯几何任务”和“body 级物理属性”分开。

`TetGeometry` 持有：

- `rest_positions`
- `tets`
- `Dm_inv`
- `rest_volumes`

`TetBody` 再补上：

- `info`
- `vertex_masses`
- `density`
- `youngs_modulus`
- `poisson_ratio`
- `fixed_vertices`

## 参考构型预计算

对一个四面体 `e = (v_0, v_1, v_2, v_3)`，参考构型边矩阵写成

$$
D_m = [X_1 - X_0,\; X_2 - X_0,\; X_3 - X_0]
$$

当前构型边矩阵写成

$$
D_s = [x_1 - x_0,\; x_2 - x_0,\; x_3 - x_0]
$$

线性四面体中的形变梯度满足

$$
F D_m = D_s
\quad \Longrightarrow \quad
F = D_s D_m^{-1}
$$

因为 `D_m` 只由参考构型决定，所以 `TetGeometry::precompute_rest_data()` 会先把 `D_m^{-1}` 预计算出来，后续每步只需要更新 `D_s`。

## 参考体积

同一个 `D_m` 还可以直接给出参考体积：

$$
V_e = \frac{|\det(D_m)|}{6}
$$

原因是 `|\det(D_m)|` 表示三条 tet 边围成的平行六面体体积，而四面体正好是它的六分之一。

如果 `\det(D_m)` 数值上接近 0，就说明这个 tet 退化了，预计算会直接拒绝。

## Lumped 顶点质量

`TetBody::precompute()` 会把单元体积转成 lumped 顶点质量：

$$
m_a = \sum_{e \ni a} \frac{\rho V_e}{4}
$$

也就是每个 tet 的总质量 `\rho V_e` 平均分给它的四个顶点。结果保存在 `vertex_masses` 中，后面再写入 `IPCState::mass_diag`。

## 为什么 `fixed_vertices` 放在 `TetBody`

`fixed_vertices` 表示顶点是否被 Dirichlet 约束固定。它是 body 级边界条件，而不是几何属性，所以应该和材料、质量一起留在 `TetBody`，而不是塞进 `TetGeometry`。

## Block 生成工具

这个文件还提供了：

- `generate_tet_geometry_block(...)`
- `generate_tet_block(...)`

它们会构造一个规则笛卡尔 block，并把每个 cube cell 固定切成 6 个四面体。

### Step 1：生成规则格点

当输入 cell 数是 `(n_x, n_y, n_z)` 时，规则格点总数为

$$
(n_x + 1)(n_y + 1)(n_z + 1)
$$

每个格点 `(i, j, k)` 都会映射到 `rest_positions` 里的一个一维下标。

### Step 2：每个 cube 切成 6 个 tet

每个 cube 会贡献 6 个四面体。实现里固定使用体对角线 `(v_{000}, v_{111})` 作为共同骨架，再围绕这条对角线展开 6 个 tet。这个切分规则简单、稳定，也很适合测试。

### Step 3：修正局部 orientation

每生成一个 tet，代码都会检查一次 `\det(D_m)` 的符号。如果 orientation 是负的，就交换两个局部顶点，保证这个 tet 在参考构型下具有正体积取向，避免后续预计算被“只是顶点顺序反了”这种问题卡住。

## 这个文件的意义

`tet_body.hpp` 是 FEM 连续理论第一次真正落成引擎数据结构的地方：

- 一个 3D 实体被表示成共享顶点和 tet connectivity 的体网格
- 参考构型量只预计算一次
- body 级质量和边界条件附着在几何之上，但不污染纯几何代码
