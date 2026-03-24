# `point_edge_distance.hpp`

`src/rtr/system/physics/ipc/geometry/point_edge_distance.hpp` 实现了点到线段的距离平方。

## 理论

给定点 $p$ 和线段端点 $e_0, e_1$，先记边方向为：

$$
u = e_1 - e_0
$$

再把点投影到无穷直线上：

$$
\alpha = \frac{(p - e_0)\cdot u}{u \cdot u}
$$

线段距离分成三个区域：

1. 当 $\alpha \le 0$ 时，对应 `Endpoint0`
2. 当 $\alpha \ge 1$ 时，对应 `Endpoint1`
3. 当 $0 < \alpha < 1$ 时，对应 `EdgeInterior`

### 局部 DOF 排列

PE 使用 9 个局部 DOF：

```text
[p_x, p_y, p_z, e0_x, e0_y, e0_z, e1_x, e1_y, e1_z]
```

## 内部区公式

在内部区，点到直线的距离平方可以写成：

$$
s(p, e_0, e_1) =
\frac{\|(e_1 - e_0) \times (p - e_0)\|^2}{\|e_1 - e_0\|^2}
$$

当前实现对这个光滑表达式直接做二阶 AutoDiff 求值。

## 代码里的区域处理方式

实现路径是按几何逻辑拆开的，而不是塞进一个大闭式函数：

1. 先检查 `p`、`e0`、`e1` 是否 finite
2. 显式拒绝零长度边
3. 计算 `alpha`
4. 再根据区域分支

### 端点区

如果投影落在线段外部，最近点就是某个端点。代码直接复用 `PointPointDistance`：

```cpp
const auto pp = PointPointDistance::compute({.p0 = input.p, .p1 = input.e0});
detail::embed_distance_result<2, 3>(pp, {0, 1}, result);
```

终点 1 的情况只是映射不同。

这意味着 PE 不需要再重复写一套 PP 的导数逻辑。

### 内部区

如果投影落在线段内部，代码就把 9 个局部 DOF 打包起来，通过 `evaluate_distance_expression<9>(...)` 对内部区表达式求值。

## Embedding 规则

端点 fallback 使用的是 PP 的 6 DOF 结果，需要嵌回 PE 的 9 DOF 布局：

- `{0, 1}` 表示 `[p, e0]`
- `{0, 2}` 表示 `[p, e1]`

没有参与的那个端点，其 gradient 和 Hessian block 都保持为零。

## 退化边策略

当前实现对零长度边采用显式报错：

```cpp
if (edge_length_squared <= detail::kMinEdgeLengthSquared) {
    throw std::invalid_argument(...);
}
```

这是有意为之。退化边不会被悄悄当成“距离为 0”，否则几何错误会被带入 barrier 层并污染后续求解。

## 在系统中的作用

PE 是 geometry 层第一个真正的组合 kernel：

- PT 的边区复用 PE
- EE 的 fallback 区域复用 PE
- CCD 和 tests 也都能通过统一的 `Distance` 接口消费它

因此 PE 首次完整体现了这套设计模式：

- 先分类
- 只对 smooth region 求值
- 通过复用子核处理退化区，而不是把所有情况塞进一个表达式
