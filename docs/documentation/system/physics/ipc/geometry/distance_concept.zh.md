# `distance_concept.hpp`

`src/rtr/system/physics/ipc/geometry/distance_concept.hpp` 定义了 IPC 局部距离核的编译期接口约束。

## 设计动机

后续 IPC 接触层会消费多种局部 primitive：

- point-point
- point-edge
- point-triangle
- edge-edge

这些 kernel 会被多个上层模块复用：

- barrier/contact 装配
- CCD
- 有限差分导数测试

如果没有统一接口，高层代码就不得不为每个 kernel 分别写适配：

- PT 可能返回一个 struct
- EE 可能返回一个 tuple
- PE 可能只暴露一个自由函数
- PP 又可能采用另一套命名

这样几何层的分支就会泄漏到更高层。

## 解决方案：类内 `Input` + `Result` + `compute(...)`

每个距离核都定义：

- 一个嵌套的 `Input`
- 一个嵌套的 `Result`
- 一个 `static compute(const Input&)`

concept 只约束这套共同外形：

```cpp
template <typename T>
concept Distance = requires(const typename T::Input& input,
                            const typename T::Result& result) {
    typename T::Input;
    typename T::Result;
    { T::compute(input) } -> std::same_as<typename T::Result>;
    { result.distance_squared } -> std::convertible_to<double>;
    result.gradient;
    result.hessian;
};
```

## 为什么 concept 不固定维度

局部几何 kernel 的 DOF 数并不统一：

- PP 是 6
- PE 是 9
- PT 是 12
- EE 是 12

因此 concept 只检查是否存在：

- `distance_squared`
- `gradient`
- `hessian`

而不强行约束具体的 Eigen 维度。

## 当前满足 concept 的 kernel

当前已覆盖：

- `PointPointDistance`
- `PointEdgeDistance`
- `PointTriangleDistance`
- `EdgeEdgeDistance`

这已经足够支撑当前 geometry 层，以及后续建立在其上的 contact / CCD 阶段。

## 统一接口的好处

1. **Barrier 层可以写成泛型**
   - 未来代码只要拿到任意 `Distance` kernel，都能沿同一套 `Input -> Result` 协议消费

2. **CCD 可以直接复用这批 kernel**
   - 一个局部 pair evaluator 只需要能构造 `Input(alpha)` 并调用 `D::compute(...)`

3. **测试可以共用一套 FD helper**
   - 同一个 finite-difference harness 可以统一打包输入、调用 `compute(...)`、比较导数

## 与 `Energy` concept 的区别

`Distance` 在设计思路上和已有的 `Energy` concept 相近，但服务的层次不同：

| | `Energy` | `Distance` |
|---|---|---|
| 操作空间 | 全局 DOFs | 局部 primitive DOFs |
| 返回形式 | energy + 分开的 gradient / sparse Hessian triplets 接口 | 一个局部 `Result`，一次返回 dense 导数 |
| Hessian 形式 | Sparse triplets | Dense 固定维度矩阵 |
| 主要消费方 | IPC system / Newton solver | Barrier / CCD / 导数测试 |

几何层本质上是局部 dense 求值单元，因此用一个完整的局部 `Result` 会比照搬全局能量那种三函数接口更自然。
