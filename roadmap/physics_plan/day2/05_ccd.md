# Phase 5: Continuous Collision Detection (CCD)

## 目标

实现 point-triangle 和 edge-edge 的连续碰撞检测，用于约束 Newton line search 的步长，保证每一步更新都不会穿透。

## 为什么 barrier 不够

barrier 只保证**当前位置**的距离不为零。但在 Newton 迭代中，一次更新 $x \leftarrow x + \alpha \Delta x$ 可能跳过 barrier 区间（从 $d > \hat{d}$ 直接到 $d < 0$）。

CCD 的作用是：给定当前位置 $x$ 和搜索方向 $\Delta x$，计算最大安全步长 $\alpha_{max}$，使得对所有 $\alpha \in [0, \alpha_{max}]$，没有任何原语对相交。

然后 line search 在 $[0, \alpha_{max}]$ 上做 Armijo 回溯，保证：
1. 充分下降（Armijo 条件）
2. 无穿透（$\alpha \leq \alpha_{max}$）

---

## CCD 的数学基础

### Point-Triangle CCD

给定：
- 点 $p$ 从 $p^0$ 移动到 $p^1 = p^0 + \alpha \Delta p$
- 三角形 $(t_0, t_1, t_2)$ 从 $(t_0^0, t_1^0, t_2^0)$ 移动到对应的 $t_i^1 = t_i^0 + \alpha \Delta t_i$

在 $\alpha \in [0, 1]$ 上，四个点做线性插值。如果在某个 $\alpha = \alpha^*$ 时点落在三角形上，就是一次碰撞。

数学上等价于求解：

$$p(\alpha) = (1-u-v) \cdot t_0(\alpha) + u \cdot t_1(\alpha) + v \cdot t_2(\alpha)$$

且 $u \geq 0, v \geq 0, u+v \leq 1$。

展开后变成求一个关于 $(\alpha, u, v)$ 的三元方程组中的最小正根。

### Edge-Edge CCD

类似地，给定两条边各自的运动轨迹，在 $\alpha \in [0, 1]$ 上求最早相交时刻。

---

## 实现策略：cubic equation solver

PT-CCD 最终归结为一个关于 $\alpha$ 的三次方程（共面条件）：

$$\det[p(\alpha) - t_0(\alpha),\ t_1(\alpha) - t_0(\alpha),\ t_2(\alpha) - t_0(\alpha)] = 0$$

步骤：
1. 展开 determinant 得到 $\alpha$ 的三次多项式
2. 求所有实根
3. 对每个根 $\alpha^* \in (0, 1]$，检查 $(u, v)$ 是否在三角形内
4. 返回最小的合法 $\alpha^*$

Edge-Edge CCD 也是类似的三次方程。

### 数值稳定性注意

- 三次方程求解使用 Cardano 公式或 eigenvalue method
- 根的过滤需要宽容的 epsilon（$10^{-6}$）
- conservative padding：返回 $\alpha_{max} = (1 - \epsilon) \cdot \alpha^*$，留安全裕度

---

## 推荐的第一版实现

Day 2 不需要做最优 CCD。推荐第一版用 **conservative bisection CCD**，比 cubic solver 更简单且更鲁棒：

### Bisection CCD 算法

```
function ccd_bisection(x, dx, collision_mesh, max_iterations=64):
    alpha_lo = 0
    alpha_hi = 1

    for iter in range(max_iterations):
        alpha_mid = (alpha_lo + alpha_hi) / 2
        x_mid = x + alpha_mid * dx

        if any_intersection(x_mid, collision_mesh):
            alpha_hi = alpha_mid
        else:
            alpha_lo = alpha_mid

    return alpha_lo * (1 - 1e-6)  # conservative
```

其中 `any_intersection` 检查是否有任何原语对距离为零或为负。

但 bisection 只适合检测"在某个 alpha 是否有穿透"，不能精确找到首次碰撞时间。更好的方案是：

### 推荐：AABB-based conservative step size

1. 对每个 PT/EE 候选对，计算运动轨迹的 AABB
2. 如果 AABB 不重叠，该对不可能碰撞，跳过
3. 对 AABB 重叠的对，用二分法或解析法求首次碰撞的 $\alpha$
4. 返回所有候选对中最小的 $\alpha_{max}$

### 最简版本（Day 2 推荐）

直接用最小距离法做 conservative 估计：

```cpp
double compute_collision_free_stepsize(
    const Eigen::VectorXd& x,
    const Eigen::VectorXd& dx,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    double min_distance = 1e-6,
    std::size_t max_iterations = 20
) {
    double alpha = 1.0;

    for (std::size_t iter = 0; iter < max_iterations; ++iter) {
        Eigen::VectorXd x_trial = x + alpha * dx;
        double d_min = compute_min_distance(x_trial, collision_mesh, candidates);

        if (d_min > min_distance) {
            break;  // safe
        }
        alpha *= 0.5;  // backtrack
    }

    return alpha;
}
```

这不是真正的 CCD（不求精确碰撞时间），但对 Day 2 足够用。真正的 cubic solver CCD 可以留到后续优化。

---

## 集成到 line search

当前 `line_search.hpp` 的 `backtracking_line_search` 接受 energy function 和初始 alpha。修改方式：

```cpp
// 在 Newton solver 的 line search 前，先求 CCD 安全步长
double alpha_ccd = compute_collision_free_stepsize(state.x, dx, collision_mesh, candidates);

// line search 的初始 alpha 不能超过 CCD 安全步长
double alpha_init = std::min(1.0, alpha_ccd);

// 然后做 Armijo backtracking，但 alpha 始终 <= alpha_ccd
LineSearchResult ls_result = backtracking_line_search(energy_fn, state.x, dx, alpha_init);
```

最简修改：给 `backtracking_line_search` 加一个 `alpha_max` 参数即可。

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/ccd/ccd_utils.hpp` | 最小距离计算、辅助函数 |
| `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp` | conservative 安全步长计算 |

可选（如果时间充裕做 cubic solver）：

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/ccd/point_triangle_ccd.hpp` | 解析 PT CCD |
| `src/rtr/system/physics/ipc/ccd/edge_edge_ccd.hpp` | 解析 EE CCD |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp` | 验证安全步长不允许穿透 |
