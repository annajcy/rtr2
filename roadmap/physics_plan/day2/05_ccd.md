# Phase 5: Continuous Collision Detection (CCD)

## 目标

实现 point-triangle 和 edge-edge 的连续碰撞检测，用于约束 Newton line search 的步长，保证每一步更新都不会穿透。

这里的实现目标和 Phase 1 / Phase 3 保持一致：CCD 不再直接绑死在 PT / EE 的具体函数名上，而是**通过 `Distance concept` 统一复用局部距离核**。

## 为什么 barrier 不够

barrier 只保证**当前位置**的距离不为零。但在 Newton 迭代中，一次更新 $x \leftarrow x + \alpha \Delta x$ 可能跳过 barrier 区间（从 $d > \hat{d}$ 直接到 $d < 0$）。

CCD 的作用是：给定当前位置 $x$ 和搜索方向 $\Delta x$，计算最大安全步长 $\alpha_{max}$，使得对所有 $\alpha \in [0, \alpha_{max}]$，没有任何原语对相交。

然后 line search 在 $[0, \alpha_{max}]$ 上做 Armijo 回溯，保证：
1. 充分下降（Armijo 条件）
2. 无穿透（$\alpha \leq \alpha_{max}$）

---

## CCD 应直接复用 `Distance concept`

如果 barrier 已经按 `Distance concept` 组织，而 CCD 仍然单独维护：

- 一套 PT 距离入口
- 一套 EE 距离入口
- 一套“某个 alpha 下是否穿透”的重复逻辑

那么 Day 2 很快就会出现两份几何逻辑并行演化的问题。

正确的分层应当和 barrier 类似：

```text
geometry/
    提供 PointTriangleDistance / EdgeEdgeDistance / ...

ccd/
    只关心：
    1. 给定 input(alpha)，这个局部距离是否还安全
    2. 如何从一条搜索方向中找出 conservative alpha_max

contact candidate wrapper
    负责从 CollisionMesh + x + dx 组出 Distance::Input(alpha)
```

也就是说，CCD 核心 helper 不应该直接依赖 `CollisionMesh` 或 `PTCandidate` / `EECandidate`。它应该只依赖：

- 一个满足 `Distance` concept 的距离核 `D`
- 当前状态与搜索方向如何生成 `typename D::Input`

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

## Day 2 推荐实现：基于 `Distance concept` 的 conservative CCD

Day 2 不需要一上来实现完整解析 CCD。更现实的目标是：

- 保证 line search 不跨过接触
- 让 PT / EE 共用同一套安全步长逻辑
- 让 CCD 和 barrier 一样，建立在统一的局部距离核之上

### 推荐的分层

```text
generic local CCD helper
    ^
    | 由 PT / EE wrapper 提供 input(alpha)
    |
candidate-specific bridge
    ^
    | 由 collision mesh 读取当前 x / dx
    |
compute_collision_free_stepsize(...)
```

### 局部 CCD helper 的职责

对任意 `Distance` kernel `D`：

1. 在 `alpha = 0` 和 `alpha = 1` 之间评估距离是否安全
2. 通过回退或二分找到 conservative 的安全上界
3. 返回这个 pair 的 `alpha_max`

这里“安全”最简单的判据就是：

- `distance_squared(alpha) > min_distance_squared`

而 `distance_squared(alpha)` 直接来自 `D::compute(input(alpha))`。

这意味着 CCD 的第一版完全可以复用 geometry 层，而不需要重复写一套 PT/EE 距离判断。

---

## 推荐的第一版实现

Day 2 不需要做最优 CCD。推荐第一版用 **conservative backtracking / bisection CCD**，比 cubic solver 更简单且更鲁棒。

### 泛型 local CCD 伪代码

```
function pair_ccd_backtracking(build_input_at_alpha, DistanceKernel, min_distance_sq):
    alpha = 1.0

    for iter in range(max_iterations):
        input = build_input_at_alpha(alpha)
        result = DistanceKernel.compute(input)

        if result.distance_squared > min_distance_sq:
            return alpha * (1 - eps)

        alpha *= 0.5

    return 0.0
```

这个版本不精确求首次碰撞时间，但有几个现实优势：

- 与 geometry/barrier 共用同一批距离核
- 不需要再维护一份 PT/EE 判定逻辑
- 对 Day 2 demo 足够稳健

### 如果需要更紧的上界

第一版回退法稳定但偏保守。如果 demo 中步长太小、Newton 收敛明显变慢，可以再升级到：

- “先 backtracking 找到安全区间，再二分细化”

即：

1. 先不断减半，直到找到一个安全 `alpha_safe`
2. 记录上一个不安全步长 `alpha_unsafe`
3. 在 `[alpha_safe, alpha_unsafe]` 间做二分，取保守解

这个版本依然不需要解析多项式根求解，但会比简单减半更不保守。

### 推荐：AABB-based conservative step size

1. 对每个 PT/EE 候选对，计算运动轨迹的 AABB
2. 如果 AABB 不重叠，该对不可能碰撞，跳过
3. 对 AABB 重叠的对，用二分法或解析法求首次碰撞的 $\alpha$
4. 返回所有候选对中最小的 $\alpha_{max}$

### 最简版本（Day 2 推荐）

直接用最小距离法做 conservative 估计，但底层评估统一通过 `Distance` kernel：

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
        double d_min = compute_min_distance_squared(
            x, dx, collision_mesh, candidates, alpha
        );

        if (d_min > min_distance * min_distance) {
            break;  // safe
        }
        alpha *= 0.5;  // backtrack
    }

    return alpha;
}
```

这不是真正的 CCD（不求精确碰撞时间），但对 Day 2 足够用。真正的 cubic solver CCD 可以留到后续优化。

---

## 基于 `Distance` concept 的接口设计

和 barrier 一样，CCD 层最重要的不是最外层 `compute_collision_free_stepsize(...)`，而是一个统一的 pair-level helper。

### 推荐的局部 pair CCD 接口

```cpp
struct CCDConfig {
    double min_distance_squared{1e-12};
    double conservative_rescale{1.0 - 1e-6};
    std::size_t max_iterations{20};
};

template <Distance D, typename InputBuilder>
double compute_pair_collision_free_stepsize(
    InputBuilder&& build_input_at_alpha,
    const CCDConfig& config
);
```

其中 `build_input_at_alpha(alpha)` 返回 `typename D::Input`。

内部逻辑统一为：

1. 用 `build_input_at_alpha(alpha)` 构造某个步长下的局部原语
2. `const auto result = D::compute(input);`
3. 若 `result.distance_squared > min_distance_squared`，则该步长安全
4. 否则缩小步长，直到安全或归零

### 为什么这样设计

这让 CCD 的核心只依赖两件事：

- 局部距离核 `D`
- 如何从 `alpha` 构造输入

而不需要知道：

- 点和边/面来自哪个 body
- 顶点坐标来自 `IPCState.x` 还是 obstacle
- candidate 是 PT 还是 EE

### candidate wrapper 的职责

和 barrier 一样，candidate-specific wrapper 只做桥接：

1. 从 `CollisionMesh` 读取局部顶点在 `x` 和 `x + dx` 方向下的位置
2. 构造 `alpha -> PointTriangleDistance::Input`
3. 或构造 `alpha -> EdgeEdgeDistance::Input`
4. 调用 `compute_pair_collision_free_stepsize<D>(...)`

于是：

- PT / EE 的差异只在 wrapper
- CCD 数学逻辑只写一遍

### 对 obstacle 的处理

这里 `Distance concept` 的好处也很直接：

- deformable 顶点的位置是 `x + alpha * dx`
- obstacle 顶点的位置是常量

但这不会改变 `typename D::Input` 的形状，只会影响 `build_input_at_alpha(alpha)` 的实现。

所以 obstacle 完全不需要在 CCD 核心层做特殊分支。

---

## `compute_min_distance_squared(...)` 也应做成泛型复用

当前文档里的 `compute_min_distance(...)` 容易再次走回“PT 一套、EE 一套”的老路。

更合理的定义应当是：

```cpp
template <Distance D, typename InputBuilderRange>
double compute_min_distance_squared_over_pairs(
    InputBuilderRange&& builders,
    double alpha
);
```

然后最外层 `compute_collision_free_stepsize(...)` 遍历：

- 所有 PT builder
- 所有 EE builder

对每个 builder 在当前 `alpha` 下调用 `D::compute(...)`，统一取最小 `distance_squared`。

这样 CCD 模块内部真正依赖的“几何求值”永远只有 `Distance::compute(...)`。

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
| `src/rtr/system/physics/ipc/geometry/distance_concept.hpp` | CCD 会复用的统一距离核接口 |
| `src/rtr/system/physics/ipc/ccd/ccd_utils.hpp` | 基于 `Distance` concept 的 pair/local CCD helper |
| `src/rtr/system/physics/ipc/ccd/collision_free_stepsize.hpp` | candidate wrapper + 全局 conservative 安全步长 |

可选（如果时间充裕做 cubic solver）：

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/ccd/point_triangle_ccd.hpp` | 解析 PT CCD |
| `src/rtr/system/physics/ipc/ccd/edge_edge_ccd.hpp` | 解析 EE CCD |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/ccd/collision_free_stepsize_test.cpp` | 验证安全步长不允许穿透 |

### 建议的测试分层

1. 局部 pair CCD helper
   - `compute_pair_collision_free_stepsize<PointTriangleDistance>`
   - `compute_pair_collision_free_stepsize<EdgeEdgeDistance>`
2. candidate wrapper
   - obstacle 顶点常量、deformable 顶点随 `alpha` 变化时，输入构造正确
3. 全局安全步长聚合
   - 多个候选对时返回最小 `alpha_max`
   - 返回的 `alpha_max` 真正能让所有 pair 保持 `distance_squared > min_distance_squared`

这样 CCD 的测试结构也会和 geometry / barrier 的 concept-based 分层一致。
