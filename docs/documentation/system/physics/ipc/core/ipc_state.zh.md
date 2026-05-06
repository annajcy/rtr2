# `ipc_state.hpp`

`src/rtr/system/physics/ipc/core/ipc_state.hpp` 定义了 `IPCState`，它是未来 FEM/IPC 求解器的全局自由度容器。

## 为什么需要一个全局向量

连续系统最初描述的是位移场 $x(X, t)$。线性有限元会用节点值和 shape function 来近似这个无限维函数：

$$
\hat{x}(X) = \sum_{a=1}^{N} x_a N_a(X)
$$

其中 $x_a \in \mathbb{R}^3$ 是节点 $a$ 的坐标。做完离散以后，所有节点未知量会被堆叠成一个全局向量：

$$
\mathbf{x} =
\begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_N
\end{bmatrix}
\in \mathbb{R}^{3N}
$$

`IPCState::x` 存的就是这组未知量。

## 布局

`IPCState` 采用 vertex-major 布局：

```text
x = [x0x, x0y, x0z, x1x, x1y, x1z, ..., x(N-1)z]^T
```

所以顶点 `i` 的起始下标是 `3 * i`，`position(i)` 本质上就是 `x.segment<3>(3 * i)`。

同样的布局也用于：

- `x_prev`
- `v`
- `mass_diag`

## 字段含义

- `x`：当前节点坐标
- `x_prev`：上一步节点坐标
- `v`：节点速度
- `mass_diag`：lumped 对角质量，直接按 $3N$ 向量存储

如果顶点 `i` 的标量质量是 $m_i$，那么对应的对角质量矩阵写成：

$$
\mathbf{M} = \mathrm{diag}(m_1, m_1, m_1, m_2, m_2, m_2, \ldots, m_N, m_N, m_N)
$$

这就是为什么 `mass_diag` 会把同一个顶点质量重复三次。

## API 取舍

`resize(vertex_count)` 会把四个向量统一 resize 成 `3 * vertex_count` 并清零。

`vertex_count()` 和 `position(i)` 现在使用的是 debug 下的 `assert`，而不是异常。原因是这个容器后面会处在热路径里，更合理的策略是由 `resize()` 建立不变量，访问器在 release 下尽量轻量。

## 为什么这种表示重要

后续所有能量、梯度和 Hessian 都会围绕同一个全局变量写。一个典型的惯性能写法是：

$$
E_I(\mathbf{x}) = \frac{1}{2h^2}(\mathbf{x} - \hat{\mathbf{x}})^T \mathbf{M} (\mathbf{x} - \hat{\mathbf{x}})
$$

当质量矩阵采用 lumped 对角形式时，这个式子就退化成逐自由度求和，所以 `IPCState` 当前直接保存向量，而不是先构造一个稀疏质量矩阵。
