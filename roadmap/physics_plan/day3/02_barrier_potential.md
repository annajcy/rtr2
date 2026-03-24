# Phase 2: Barrier Function + Potential

## 目标

实现 IPC 的 barrier energy 模块，使接触力从优化框架中涌现。

这是 Day 3 最核心的新增模块，也是 Day 2 plan 中 Phase 3 的直接执行。

## 分为两层

### 层 1: `barrier_function.hpp` — 标量 barrier

纯数学函数，不涉及几何或 DOF：

$$b(s, \hat{s}) = \begin{cases} -(s - \hat{s})^2 \ln(s / \hat{s}) & \text{if } 0 < s < \hat{s} \\ 0 & \text{if } s \geq \hat{s} \end{cases}$$

其中 $s$ 是距离平方，$\hat{s}$ 是激活阈值（距离平方）。

接口：

```cpp
namespace barrier {
    double energy(double s, double s_hat);
    double gradient(double s, double s_hat);   // db/ds
    double hessian(double s, double s_hat);    // d²b/ds²
}
```

一阶导：
$$\frac{\partial b}{\partial s} = -(s - \hat{s})\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right)$$

二阶导：
$$\frac{\partial^2 b}{\partial s^2} = -2\ln(s/\hat{s}) - \frac{(s - \hat{s})(3s - \hat{s})}{s^2}$$

数值保护：
- `s <= 0` → 返回极大值或 assert
- `s >= s_hat` → 返回 0

### 层 2: `barrier_potential.hpp` — 全局装配

基于 Day 2 已建立的 `Distance` concept，实现：

1. **泛型 pair barrier**：对任意 `Distance` kernel，计算单对的 barrier energy/gradient/Hessian
2. **链式法则**：从标量 barrier 到局部距离核空间
3. **PSD projection**：每个局部 Hessian 截断负特征值
4. **全局装配**：通过 `global_dof_map` 把局部贡献散装到全局 gradient/triplets

```cpp
struct BarrierPotentialConfig {
    double dhat_squared{0.01};
    double kappa{1e4};
    bool project_hessian_to_psd{true};
};

// 全局接口
double compute_total_barrier_energy(
    const Eigen::VectorXd& x,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config
);

void accumulate_total_barrier_gradient(
    const Eigen::VectorXd& x,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    Eigen::VectorXd& gradient
);

void accumulate_total_barrier_hessian_triplets(
    const Eigen::VectorXd& x,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    std::vector<Eigen::Triplet<double>>& triplets
);
```

### 装配流程（每个候选对）

1. 从 `CollisionMesh` + `x` 读取局部顶点位置，构造 `Distance::Input`
2. `Distance::compute(input)` → 局部 `distance_squared` + `gradient` + `hessian`
3. 若 `distance_squared >= dhat_squared` → 跳过
4. 标量 barrier：`b(s)`, `b'(s)`, `b''(s)`
5. 链式法则：
   - 局部 gradient = `kappa * b'(s) * distance_gradient`
   - 局部 Hessian = `kappa * (b''(s) * distance_gradient * distance_gradient^T + b'(s) * distance_hessian)`
6. PSD projection：特征分解 → 截断负特征值 → 重组
7. `global_dof_map` 散装到全局

### `global_dof_map` 语义

```cpp
std::span<const std::optional<Eigen::Index>> global_dof_map
```

- 有值：该局部坐标分量对应的全局 DOF 索引
- `nullopt`：obstacle 侧，不装配

这样 barrier 数学层不需要感知 obstacle vs deformable。

## 测试

### `barrier_function_test.cpp`

- 标量 barrier `b(s)` 在 `s ∈ (0, s_hat)` 范围内 gradient FD check
- 标量 barrier `b(s)` 在 `s ∈ (0, s_hat)` 范围内 Hessian FD check
- 边界行为：`b(s_hat) = 0`, `b'(s_hat) = 0`
- `s >= s_hat` 时返回 0

### `barrier_potential_test.cpp`

- `compute_pair_barrier<PointTriangleDistance>` gradient FD check
- `compute_pair_barrier<EdgeEdgeDistance>` gradient FD check
- obstacle 顶点对应的 `nullopt` DOF 不写入全局梯度
- deformable 顶点贡献正确落到全局向量

## 参数建议

| 参数 | Day 3 demo 推荐值 | 说明 |
|------|-------------------|------|
| `dhat_squared` | `0.01` ($\hat{d} \approx 0.1$) | 适合 spacing 0.2~0.3 的 tet block |
| `kappa` | `1e3 ~ 1e5` | 从小开始调 |

## 输出

- `src/rtr/system/physics/ipc/contact/barrier_function.hpp`
- `src/rtr/system/physics/ipc/contact/barrier_potential.hpp`
- `test/system/physics/ipc/contact/barrier_function_test.cpp`
- `test/system/physics/ipc/contact/barrier_potential_test.cpp`
