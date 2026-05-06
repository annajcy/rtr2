# Phase 4: CCD + 安全步长

## 目标

实现 conservative 的碰撞安全步长计算，约束 Newton line search 不穿透。

## 为什么 barrier 不够

Barrier 只保证当前位置的距离非零。但 Newton 更新 $x \leftarrow x + \alpha \Delta x$ 可能跳过 barrier 区间，从 $d > \hat{d}$ 直接到 $d < 0$。

CCD 提供安全步长 $\alpha_{max}$，保证 $\forall \alpha \in [0, \alpha_{max}]$，没有穿透。

## 实现策略：conservative backtracking CCD

Day 3 不做解析 cubic solver CCD。使用基于 `Distance` concept 的 backtracking 方法：

### 核心思路

对每个候选对，在搜索方向上不断减半步长，直到所有候选对的距离都安全：

```
function compute_collision_free_stepsize(x, dx, mesh, candidates, config):
    alpha = 1.0
    for iter in range(max_iterations):
        // 检查所有候选对在 x + alpha * dx 位置下的最小距离
        min_dist_sq = compute_min_distance_squared_at(x, dx, alpha, mesh, candidates)
        if min_dist_sq > config.min_distance_squared:
            return alpha * config.conservative_rescale
        alpha *= 0.5
    return alpha  // fallback
```

### 基于 Distance concept 的分层

和 barrier 一样，CCD 不直接绑死在 PT/EE 上：

```text
geometry/
    PointTriangleDistance / EdgeEdgeDistance（已完成）

ccd/
    泛型 pair CCD helper：
    - 给定 input(alpha)，评估距离是否安全
    - 通过 D::compute(input) 复用距离核

    全局安全步长：
    - 遍历所有 PT/EE 候选，取最小 alpha_max
```

### CCD 配置

```cpp
struct CCDConfig {
    double min_distance_squared{1e-12};
    double conservative_rescale{1.0 - 1e-6};
    std::size_t max_iterations{20};
};
```

### 对 obstacle 的处理

- deformable 顶点：位置为 `x + alpha * dx`
- obstacle 顶点：位置为常量

这不改变 `Distance::Input` 的形状，只影响 `build_input_at_alpha(alpha)` 的实现。CCD 核心层不需要特殊分支。

## 集成到 line search

### `line_search.hpp` 修改

```cpp
LineSearchResult backtracking_line_search(
    // ... existing params ...
    double alpha_max = 1.0  // 新增：CCD 安全步长上界
);
```

初始 alpha 从 `min(1.0, alpha_max)` 开始。

### `newton_solver.hpp` 修改

`NewtonProblem` 新增：

```cpp
std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> compute_max_stepsize{};
```

如果非空，solver 在 line search 前调用以获取 `alpha_max`。

## 测试

### `collision_free_stepsize_test.cpp`

1. 单对 PT：点从远处向三角形运动，返回的 `alpha_max` 不允许穿透
2. 单对 EE：两条边从远处相向运动，返回的 `alpha_max` 不允许穿透
3. 多对候选：返回全局最小的 `alpha_max`
4. 已经安全的情况：`alpha_max = 1.0`
5. obstacle 顶点常量、deformable 顶点随 alpha 变化

## 输出

新增：
- `src/rtr/system/physics/ipc/ccd/ccd_config.hpp`
- `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp`
- `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp`

修改：
- `src/rtr/system/physics/ipc/solver/line_search.hpp`
- `src/rtr/system/physics/ipc/solver/newton_solver.hpp`
