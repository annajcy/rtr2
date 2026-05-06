# `point_point_distance.hpp`

`src/rtr/system/physics/ipc/geometry/point_point_distance.hpp` 实现了最小的局部距离核：两个点之间的距离平方。

## 理论

对两个点 $p_0, p_1 \in \mathbb{R}^3$：

$$
s(p_0, p_1) = \|p_0 - p_1\|^2
$$

它是整个 geometry 层的基准核。

### 局部 DOF 排列

PP 使用 6 个局部 DOF：

```text
[p0_x, p0_y, p0_z, p1_x, p1_y, p1_z]
```

### 梯度

记 $d = p_0 - p_1$，则

$$
\nabla_{p_0} s = 2d, \qquad \nabla_{p_1} s = -2d
$$

完整局部梯度可以写成：

$$
\nabla s =
\begin{bmatrix}
2(p_0 - p_1) \\
-2(p_0 - p_1)
\end{bmatrix}
$$

### Hessian

Hessian 是常量：

$$
\nabla^2 s =
\begin{bmatrix}
2I & -2I \\
-2I & 2I
\end{bmatrix}
$$

这里不存在区域切换，也不存在退化分支。

## 接口

```cpp
struct PointPointDistance {
    struct Input {
        Eigen::Vector3d p0;
        Eigen::Vector3d p1;
    };

    using Result = PointPointDistanceResult;

    static Result compute(const Input& input);
};
```

另外还提供了一个很薄的语法糖包装：

```cpp
PointPointDistanceResult point_point_distance(const Eigen::Vector3d& p0,
                                             const Eigen::Vector3d& p1);
```

## 代码实现

实现刻意保持最小化：

1. 先检查两个点都是 finite
2. 把它们打包成一个 6 DOF 局部向量
3. 用 `evaluate_distance_expression<6>(...)` 求值 $\|p_0 - p_1\|^2$

核心表达式是：

```cpp
const auto p0 = x.template segment<3>(0);
const auto p1 = x.template segment<3>(3);
return detail::squared_norm(p0 - p1);
```

虽然 PP 的 gradient/Hessian 可以直接写成闭式，但代码仍然统一走共享 AutoDiff helper，这样所有 kernel 的导数路径保持一致。

## 为什么 PP 不需要 epsilon 分支

PP 是唯一一个完全不需要几何分类的距离核：

- 没有投影参数
- 没有端点区 / 内部区切换
- 没有退化几何分支

因此 PP 也是最适合作为导数测试基准的 kernel。

## 在系统中的作用

PP 并不只是一个教学例子：

- PE 的端点区域会退化到 PP
- PT 的顶点区域会退化到 PP
- 退化 PT 会比较多个 PP candidate
- 它也是局部解析导数验证时最小、最干净的 FD 基准核
