# Phase 3: Barrier Potential

## 目标

实现 IPC 的 barrier energy，使接触力自然地从优化框架中涌现，而不是显式的碰撞检测 + 碰撞响应。

## IPC barrier 的核心思想

传统碰撞处理：检测穿透 → 施加罚力或约束力。问题在于检测和响应是两个独立步骤，容易出现"刚好穿过检测窗口"的情况。

IPC 的做法：把接触势能加入总优化目标。当两个物体靠近时，barrier energy 快速增大，阻止优化器选择穿透方向。

$$E_{total}(x) = E_{inertial}(x) + E_{gravity}(x) + \Psi_{elastic}(x) + \kappa \sum_{(i,j) \in \mathcal{C}} b(d_{ij}(x), \hat{d})$$

其中 $\kappa$ 是 barrier stiffness，$\hat{d}$ 是激活距离阈值，$b$ 是标量 barrier 函数。

---

## 标量 Barrier 函数

### 定义

IPC 论文使用的 barrier 函数：

$$b(d, \hat{d}) = \begin{cases} -(d - \hat{d})^2 \ln(d / \hat{d}) & \text{if } 0 < d < \hat{d} \\ 0 & \text{if } d \geq \hat{d} \end{cases}$$

其中 $d$ 是距离平方（$d = d^2_{physical}$，或者用物理距离 $d_{physical}$，取决于实现选择）。

**重要选择**：IPC 原始论文中 $d$ 是距离（不是距离平方）。但很多实现用距离平方作为 barrier 输入以避免开方。Day 2 推荐**使用距离平方**，这样和 Phase 1 的距离函数直接对接。

如果使用距离平方，令 $s = d^2$：

$$b(s, \hat{s}) = \begin{cases} -(s - \hat{s})^2 \ln(s / \hat{s}) & \text{if } 0 < s < \hat{s} \\ 0 & \text{if } s \geq \hat{s} \end{cases}$$

### 一阶导数

$$\frac{\partial b}{\partial s} = -(s - \hat{s})\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right)$$

### 二阶导数

$$\frac{\partial^2 b}{\partial s^2} = -\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right) - (s - \hat{s})\left(\frac{2}{s} + \frac{\hat{s}}{s^2}\right)$$

化简：

$$\frac{\partial^2 b}{\partial s^2} = -2\ln(s/\hat{s}) - \frac{(s - \hat{s})(3s - \hat{s})}{s^2}$$

### 数值注意

- 当 $s \to 0$ 时 $b \to +\infty$，$b' \to -\infty$（阻止穿透）
- 当 $s = \hat{s}$ 时 $b = 0$，$b' = 0$（$C^1$ 连续过渡到零）
- 当 $s > \hat{s}$ 时 $b = 0$（不激活）

实现中需要防止 $s \leq 0$ 进入 barrier（这是物理上不应发生的，但数值上可能出现浮点误差）：

```cpp
if (s <= 0.0) {
    // 已经穿透，返回极大值或 throw
}
```

---

## Barrier 层应直接建立在 `Distance concept` 之上

在 Day 2 之前，barrier 最容易写成两套平行代码：

- 一套 `compute_pt_barrier_*`
- 一套 `compute_ee_barrier_*`

这种写法短期能跑，但会立刻带来几个问题：

1. PT 和 EE 的链式法则逻辑完全重复
2. 后续如果 PE / PP 也要用于 CCD 或调试，很难复用
3. 测试会变成“按原语复制粘贴”

既然 [01_geometry_distances.md](/Users/jinceyang/Desktop/codebase/graphics/rtr2/roadmap/physics_plan/day2/01_geometry_distances.md) 已经决定引入 `Distance` concept，那么 barrier 层就不应该再直接绑死在 PT/EE 的具体实现上，而应当把“距离核 + 标量 barrier + 局部到全局装配”拆开。

### barrier 层的推荐分层

```text
barrier_function.hpp
    只负责标量 b(s, s_hat), b'(s), b''(s)

barrier_potential.hpp
    1. 用 Distance kernel 计算局部 distance result
    2. 用链式法则组装局部 barrier gradient / Hessian
    3. 做 PSD projection
    4. 通过局部->全局映射装配到 global gradient / triplets
```

### 推荐的泛型思路

对于任意满足 `Distance` concept 的距离核 `D`：

```cpp
template <Distance D>
struct PairBarrierResult {
    double energy{0.0};
    typename D::Result::Gradient gradient{};
    typename D::Result::Hessian hessian{};
    bool active{false};
};
```

不过考虑到当前仓库里 `Result` 还没有把 `Gradient/Hessian` 单独 typedef 出来，更现实的第一版可以直接从 `result.gradient` / `result.hessian` 的具体类型推导，不强行把模板系统做得太重。

重点不在“写出最花哨的泛型”，而在于接口层明确规定：

- barrier 不直接知道自己在处理 PT 还是 EE
- 它只知道输入是一个 `Distance` kernel 的 `Input`
- kernel 返回 `distance_squared + gradient + hessian`

---

## 从标量 barrier 到全局 DOF

### 链式法则

barrier 对全局 DOF $x$ 的贡献通过链式法则：

$$\frac{\partial B}{\partial x} = \kappa \sum_{(i,j)} \frac{\partial b}{\partial s} \cdot \frac{\partial s}{\partial x}$$

$$\frac{\partial^2 B}{\partial x^2} = \kappa \sum_{(i,j)} \left(\frac{\partial^2 b}{\partial s^2} \cdot \frac{\partial s}{\partial x} \cdot \frac{\partial s}{\partial x}^T + \frac{\partial b}{\partial s} \cdot \frac{\partial^2 s}{\partial x^2}\right)$$

其中 $s = d^2(x)$ 是距离平方，$\frac{\partial s}{\partial x}$ 和 $\frac{\partial^2 s}{\partial x^2}$ 来自 Phase 1 的几何距离模块。

### 局部到全局装配

每个接触对首先在**局部距离核空间**完成 barrier 求值，然后再装配到全局 DOF。

对 Day 2 来说，真正进入 contact candidate 的主原语仍然是：

- PT：12 DOF
- EE：12 DOF

但 barrier 层不应该依赖“它们恰好都是 12 DOF”这个事实来设计接口。更好的组织方式是：

1. 先通过 `Distance::compute(input)` 得到局部 `Result`
2. 在局部空间应用链式法则
3. 再由调用者提供的 local-to-global 映射散装配

也就是说，barrier 的核心泛型 helper 只处理“局部结果”，不处理 collision mesh 细节。

装配路径：
1. 通过 `Distance::compute(input)` 计算局部距离平方 $s$ 和局部 gradient/Hessian
2. 计算标量 barrier $b(s)$, $b'(s)$, $b''(s)$
3. 局部 barrier gradient = $\kappa \cdot b'(s) \cdot \nabla_x s$
4. 局部 barrier Hessian = $\kappa \cdot (b''(s) \cdot \nabla_x s \cdot \nabla_x s^T + b'(s) \cdot \nabla^2_x s)$
5. 对局部 Hessian 做 PSD projection
6. 通过 local-to-global DOF 映射散装到全局 gradient 和 triplet 列表

### 为什么这里适合做成泛型 helper

PT 和 EE 在 barrier 层唯一真正不同的部分是：

- 距离核输入类型不同
- local-to-global 映射构建方式不同

而下面这些逻辑完全相同：

- 标量 barrier 求值
- 链式法则
- PSD projection
- active / inactive 判断

因此 barrier 层最自然的切分应当是：

```text
generic pair barrier helper
    ^
    | 由 PT / EE candidate wrapper 提供 input 与映射
    |
collision candidate specific wrapper
```

### PSD Projection

接触 Hessian 可能不正定（距离函数的 Hessian 不一定正定）。必须对每个局部 Hessian 做 PSD 投影。

```cpp
// 特征分解
Eigen::SelfAdjointEigenSolver<LocalHessian> solver(local_hessian);
auto eigenvalues = solver.eigenvalues();
auto eigenvectors = solver.eigenvectors();

// 截断负特征值
for (int i = 0; i < eigenvalues.size(); ++i) {
    eigenvalues[i] = std::max(eigenvalues[i], 0.0);
}

// 重组
local_hessian = eigenvectors * eigenvalues.asDiagonal() * eigenvectors.transpose();
```

这保证每个接触对的 Hessian 贡献是半正定的，Newton 方向是下降方向。

---

## 参数选择

### $\hat{d}$（激活距离阈值）

| 场景 | 推荐 $\hat{d}^2$ | 说明 |
|------|------------------|------|
| Day 2 demo | `0.01` ($\hat{d} \approx 0.1$) | 适合 spacing=0.2~0.3 的 tet block |
| 一般场景 | `(0.01 \cdot bbox\_diag)^2` | 相对于场景尺度 |

### $\kappa$（barrier stiffness）

Day 2 先用固定 $\kappa$：

| 场景 | 推荐 $\kappa$ | 说明 |
|------|--------------|------|
| Day 2 demo | `1e3` ~ `1e5` | 从小开始调，太大会让 Newton 收敛变慢 |

后续可以做 adaptive $\kappa$（IPC 论文中的 Algorithm 1），但 Day 2 不做。

---

## 实现结构

```cpp
// 标量 barrier 函数（纯数学）
namespace barrier {
    double energy(double s, double s_hat);
    double gradient(double s, double s_hat);  // db/ds
    double hessian(double s, double s_hat);   // d²b/ds²
}

struct BarrierPotentialConfig {
    double dhat_squared{0.01};   // s_hat
    double kappa{1e4};
    bool project_hessian_to_psd{true};
};

template <Distance D>
struct PairBarrierResult;

template <Distance D>
PairBarrierResult<D> compute_pair_barrier(
    const typename D::Input& input,
    const BarrierPotentialConfig& config
);

template <typename LocalVector, typename LocalMatrix>
void accumulate_pair_barrier_to_global(
    const LocalVector& local_gradient,
    const LocalMatrix& local_hessian,
    std::span<const std::optional<Eigen::Index>> global_dof_map,
    Eigen::VectorXd* global_gradient,
    std::vector<Eigen::Triplet<double>>* global_triplets
);

double compute_total_barrier_energy(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config
);

void accumulate_total_barrier_gradient(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    Eigen::VectorXd& gradient  // accumulate
);

void accumulate_total_barrier_hessian_triplets(
    const Eigen::VectorXd& x,
    const CollisionMesh& collision_mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    std::vector<Eigen::Triplet<double>>& triplets  // accumulate
);
```

### 推荐的泛型 pair barrier 接口

这一步的核心不是 `compute_total_barrier_*`，而是一个可复用的 pair-level helper。

建议形状：

```cpp
template <Distance D>
struct PairBarrierResult {
    double energy{0.0};
    decltype(D::compute(std::declval<typename D::Input>()).gradient) gradient{};
    decltype(D::compute(std::declval<typename D::Input>()).hessian) hessian{};
    bool active{false};
};

template <Distance D>
PairBarrierResult<D> compute_pair_barrier(
    const typename D::Input& input,
    const BarrierPotentialConfig& config
);
```

内部逻辑统一为：

1. `const auto distance = D::compute(input);`
2. 如果 `distance.distance_squared >= dhat_squared`，返回 `active=false`
3. 否则应用标量 barrier 链式法则
4. 可选做 PSD projection

这样 PT 和 EE 的差异只体现在：

- `D = PointTriangleDistance` 或 `EdgeEdgeDistance`
- `input` 的组织方式不同

### collision candidate 到 pair barrier 的桥接

`CollisionCandidates` 里存的仍然是 `PTCandidate` / `EECandidate`。因此全局装配层需要两类很薄的 wrapper：

1. 从 candidate + `CollisionMesh` + `x` 读取几何坐标
2. 组出 `PointTriangleDistance::Input` 或 `EdgeEdgeDistance::Input`
3. 同时组出局部自由度对应的 `global_dof_map`
4. 把 `compute_pair_barrier<D>()` 的结果散装到全局

这两层拆开之后，barrier 的主复杂度会从“每种 candidate 手写一遍链式法则”变成“只为每种 candidate 手写输入映射”。

### `global_dof_map` 的推荐语义

因为 Day 2 的 obstacle 顶点不在 `IPCState.x` 内，所以局部原语的某些坐标没有全局 DOF。

最直接的办法是让装配 helper 接受：

```cpp
std::span<const std::optional<Eigen::Index>> global_dof_map
```

含义：

- 有值：对应某个全局 DOF，允许装配
- `nullopt`：对应 obstacle 侧坐标，不装配到 global system

这样同一个 pair barrier helper 可以自然支持：

- deformable vs obstacle
- deformable vs deformable
- 未来的 self-contact

而不需要在 barrier 数学层显式写“如果是 obstacle 就跳过”。

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/geometry/distance_concept.hpp` | barrier 会依赖的距离核统一接口 |
| `src/rtr/system/physics/ipc/contact/barrier_function.hpp` | 标量 $b(s, \hat{s})$ 及一二阶导数 |
| `src/rtr/system/physics/ipc/contact/barrier_potential.hpp` | 基于 `Distance` concept 的 pair barrier 与全局装配 |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/contact/barrier_function_test.cpp` | 标量 barrier fd check、边界行为 |
| `test/system/physics/ipc/contact/barrier_potential_test.cpp` | `compute_pair_barrier<PointTriangleDistance>` / `compute_pair_barrier<EdgeEdgeDistance>` 的 gradient fd check |

### 建议补的测试形状

除了原来的单对 PT/EE fd check，建议明确分三层测：

1. 标量 barrier
   - `b(s)`、`b'(s)`、`b''(s)` 的有限差分
2. 泛型 pair barrier
   - `compute_pair_barrier<PointTriangleDistance>`
   - `compute_pair_barrier<EdgeEdgeDistance>`
3. 全局装配
   - obstacle 顶点对应的 `nullopt` DOF 不会写入全局梯度/Hessian
   - deformable 顶点贡献能正确落到 `Eigen::VectorXd` 与 triplets

这样测试结构也会和新的接口分层保持一致。
