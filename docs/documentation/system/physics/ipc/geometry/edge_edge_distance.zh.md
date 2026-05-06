# `edge_edge_distance.hpp`

`src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp` 实现了两条线段之间的距离平方。

它和 PT 一起，是后续 IPC barrier 与 CCD 会直接消费的另一类核心 primitive。

## 理论

对线段 $(ea_0, ea_1)$ 和 $(eb_0, eb_1)$，最近点对大体会落在三类情况之一：

1. 两个最近点都在线段内部
2. 某个端点到另一条边最近
3. 几何上退化或近退化，需要走 fallback 逻辑

对数值稳定性来说，最危险的并不是一般位置的两条斜边，而是：

- 平行边
- 几乎平行的边
- 很短的边

因此实现结构必须围绕“安全分类 + fallback”展开。

### 局部 DOF 排列

EE 使用 12 个局部 DOF：

```text
[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]
```

## Interior-Interior 判定

记：

$$
u = ea_1 - ea_0,\qquad v = eb_1 - eb_0,\qquad w_0 = ea_0 - eb_0
$$

标准最近线参数来自：

$$
a = u\cdot u,\quad b = u\cdot v,\quad c = v\cdot v,\quad d = u\cdot w_0,\quad e = v\cdot w_0
$$

$$
s = \frac{be - cd}{ac - b^2}, \qquad
t = \frac{ae - bd}{ac - b^2}
$$

如果 `s` 和 `t` 都严格落在 `(0, 1)` 内部，代码就把当前激活区域视为 `InteriorInterior`。

### 内部区的光滑表达式

在这种情况下，当前实现用未归一化叉积表达式来评估分离距离：

$$
n = (ea_1 - ea_0) \times (eb_1 - eb_0)
$$

$$
s =
\frac{\big((ea_0 - eb_0)\cdot n\big)^2}{\|n\|^2}
$$

和 PT 面内部一样，这里故意不用单位法向量，而是让 AutoDiff 去处理未归一化表达式的导数链。

## 平行 / 近平行处理

在信任内部区公式之前，代码先检查：

$$
\|(ea_1 - ea_0) \times (eb_1 - eb_0)\|^2
$$

如果这个量小于 `kParallelThreshold`，就把当前情况视为平行或近似平行，并直接跳过内部区公式。

原因是：当两条边的方向几乎线性相关时，内部区参数求解会变得非常脆弱。

## Fallback 设计

当前 fallback 路径是显式的、几何意义清楚的：

1. 计算 `ea0 -> edge B`
2. 计算 `ea1 -> edge B`
3. 计算 `eb0 -> edge A`
4. 计算 `eb1 -> edge A`
5. 取 `distance_squared` 最小的 candidate

这些 candidate 全都是 `PointEdgeDistance` 的结果，再嵌回 EE 的 12 DOF 布局。

因此 fallback 的导数不是在 EE 里重新推出来的，而是直接来自已经经过测试的 PE kernel。

## 零长度边策略

当前实现对零长度输入边采用显式拒绝：

```cpp
if (edge_a_length_squared <= detail::kMinEdgeLengthSquared ||
    edge_b_length_squared <= detail::kMinEdgeLengthSquared) {
    throw std::invalid_argument(...);
}
```

这比 PT 的退化三角形降级更严格，因为一旦缺少一条有效边，EE 作为“边边距离核”本身就失去了清晰的几何意义。

## 当前导数策略

和 PT 一样：

- 区域选择是手写几何逻辑
- 激活的光滑表达式通过二阶 AutoDiff 求导
- fallback 区域通过复用低层 kernel 获得导数，而不是在 EE 里重复写一套

这让 EE 的实现保持了一个实用平衡：

- 对危险几何情形有显式控制
- 在光滑分支上保留解析导数
- 在退化分支上复用已验证的子核

## 测试

当前测试覆盖了：

- interior-interior
- endpoint-on-edge fallback
- 通过同一 candidate 机制覆盖的 endpoint-endpoint 风格 case
- 平行
- 几乎平行
- 短边

有限差分校验只放在代表性的光滑 case 上，而不会在 active-set 切换边界上做。

## 在后续 IPC 接触中的作用

EE 是核心接触 primitive，因为三角网格和碰撞表面经常产生 edge-edge proximity pair。

因此这个文件不只是返回一个最小距离标量：

- 它需要保持稳定的 12 DOF 局部顺序
- 它需要保留 region/debug 信息
- 它要把平行 fallback 做成一等分支
- 它还要返回后续 barrier 装配可直接使用的局部 dense 导数
