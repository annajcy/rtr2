# `distance_common.hpp`

`src/rtr/system/physics/ipc/geometry/distance_common.hpp` 放的是当前所有距离核共享的基础支撑件。

这个文件本身不是某个具体几何算法。它的职责是给 PP / PE / PT / EE 提供统一的结果形状、导数求值和结果嵌入机制。

## `DistanceResult<Dofs>`

核心类型是：

```cpp
template <int Dofs>
struct DistanceResult {
    double distance_squared;
    Eigen::Matrix<double, Dofs, 1> gradient;
    Eigen::Matrix<double, Dofs, Dofs> hessian;
};
```

它把每个距离核的返回值固定成一个定长局部对象：

- 一个标量 `distance_squared`
- 一个局部 dense gradient
- 一个局部 dense Hessian

固定维度在 IPC 接触里很重要，因为这里天然是局部求值：

- PP 是 6 个 DOF
- PE 是 9 个 DOF
- PT 和 EE 是 12 个 DOF

后续 barrier 层会把这些 dense block 再映射进全局 sparse 装配。

## `detail` 里的共享能力

### 输入和输出的 finite 检查

`require_finite_point(...)` 在进入几何求值前先检查局部坐标是否有效。

`require_finite_result(...)` 在返回前检查标量、gradient 和 Hessian 是否都是 finite。

这和当前 IPC 其他模块的风格一致：

- 非法局部几何应该尽早失败
- geometry kernel 不应把 NaN 默默传给 barrier 或 solver

### `embed_distance_result(...)`

PT 和 EE 会复用低维子核。例如：

- PT 的边区调用 PE
- PT 的顶点区调用 PP
- EE 的 fallback 区域调用 PE

这些子核返回的导数都在自己的局部坐标系里。`embed_distance_result(...)` 会根据 point index map，把参与点对应的 `3 x 3` block 拷贝回父级的更大梯度和 Hessian 里。

概念上就是：

```text
PP / PE 局部导数
    -> 点索引映射
    -> 写回 PT / EE 的 dense block
```

这样既把复用关系写清楚，也避免为端点/边区重复推导同一套公式。

### `evaluate_distance_expression(...)`

最关键的 helper 是 `evaluate_distance_expression(...)`。

它接收：

- 一个打包好的局部 DOF 向量
- 一个关于这些 DOF 的光滑标量表达式

然后用嵌套的 `Eigen::AutoDiffScalar` 同时恢复：

- 标量值
- 一阶导
- 二阶导

实现使用了两层 AutoDiff 标量：

```cpp
using InnerScalar = Eigen::AutoDiffScalar<FirstDerivative>;
using Scalar = Eigen::AutoDiffScalar<SecondDerivative>;
```

外层的一阶导项本身是可微的内层标量，因此最终可以恢复 dense Hessian，而不需要把所有二阶导手写出来。

这里仍然是**运行时解析导数路径**，不是数值差分。代码是在自动微分一条闭式表达式。

## 为什么 `cross`、`squared_norm` 和阈值常量放在这里

多个距离核都会重复用到一些很薄的几何工具：

- `squared_norm(...)` 让标量表达式保持泛型
- `cross(...)` 让这些 3D kernel 不必额外依赖更宽的 Eigen geometry 模块
- 数值阈值统一放在一起，方便集中管理退化策略

当前阈值的含义：

| 常量 | 语义 |
|---|---|
| `kMinEdgeLengthSquared` | 判定边是否接近零长度 |
| `kMinTriangleAreaSquared` | 判定三角形是否接近零面积 |
| `kParallelThreshold` | 判定边边是否平行或近似平行 |

这些常量不是物理参数，而是几何鲁棒性阈值，用来选择安全的求值路径。

## Hessian 处理原则

这个文件也间接固定了一个重要架构边界：

- geometry 层返回的是当前 active distance expression 的真实局部二阶导
- 它**不会**在这里做 PSD projection
- 也**不会**为了求解稳定性做正则化

这正是 IPC 该有的分层方式：

$$
H_{barrier} = b''(s)\nabla s \nabla s^T + b'(s)\nabla^2 s
$$

任何正定化或稳定化都应该留给 barrier / solver 层，而不是塞进 `distance_common.hpp`。
