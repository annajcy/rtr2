# `energy_concept.hpp`

`src/rtr/system/physics/ipc/energy/energy_concept.hpp` 定义了 IPC energy 模块的编译期接口约束。

## 设计动机

IPC 求解器的总能量是多个独立项的叠加：

$$
E(x) = E_{inertial}(x) + E_{gravity}(x) + E_{elastic}(x) + \ldots
$$

每个能量项都需要提供 energy / gradient / Hessian triplets 三个计算接口。但不同能量项需要的输入参数不同：

- `InertialEnergy` 需要 `x`, `x_hat`, `mass_diag`, `dt`
- `GravityEnergy` 需要 `x`, `mass_diag`, `gravity`
- `MaterialEnergy<M>` 需要 `TetBody`, `x`, `Material`

如果把所有参数塞进统一的参数列表，接口会变得臃肿且脆弱。

## 解决方案：类内 `Input` + concept

每个能量类型定义自己的 `Input` 结构体，把所需参数打包在内。`Energy` concept 只约束接口形状：

```cpp
template <typename T>
concept Energy = requires(const typename T::Input& input,
                          Eigen::VectorXd& gradient,
                          std::vector<Eigen::Triplet<double>>& triplets) {
    typename T::Input;
    { T::compute_energy(input) } -> std::convertible_to<double>;
    { T::compute_gradient(input, gradient) } -> std::same_as<void>;
    { T::compute_hessian_triplets(input, triplets) } -> std::same_as<void>;
};
```

## 各能量项的 `Input`

| 能量项 | `Input` 包含 | Hessian |
|--------|-------------|---------|
| `InertialEnergy` | `x`, `x_hat`, `mass_diag`, `dt` | 对角，正定 |
| `GravityEnergy` | `x`, `mass_diag`, `gravity` | 零（no-op） |
| `MaterialEnergy<M>` | `TetBody`, `x`, `Material` | 稀疏，per-tet 12×12 block |

## 统一接口的好处

1. **泛型装配**：未来可以写 `template <Energy E> void assemble(...)` 来统一处理
2. **零 Hessian 项不需要特判**：`GravityEnergy` 的 `compute_hessian_triplets` 是 no-op，但编译期类型检查通过
3. **新能量项只需满足 concept**：加 barrier / friction 时，只要定义 `Input` + 三个方法，自动集成

## 与 `TetMaterialModel` concept 的区别

| | `Energy` | `TetMaterialModel` |
|---|---------|---------------------|
| 操作空间 | 全局 DOF（$3n$ 向量） | 单元局部（$3 \times 3$ 的 $F$） |
| Hessian 格式 | Triplets（稀疏） | 密集 $9 \times 9$ 矩阵 |
| 桥接 | 直接被 solver 调用 | 通过 `MaterialEnergy` 桥接 |
