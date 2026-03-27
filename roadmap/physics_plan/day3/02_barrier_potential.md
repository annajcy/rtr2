# Phase 2: Barrier Function + Potential

## 先回答你现在最关心的问题

可以开始，而且现在就是合适的时机。

按当前仓库现状，这个 phase 的直接前置条件已经满足：

- `Distance` kernels 已完成：`PointPointDistance` / `PointEdgeDistance` / `PointTriangleDistance` / `EdgeEdgeDistance`
- `DistanceResult` 已统一提供 `distance_squared + gradient + hessian`
- `CollisionMesh` 已能从 `IPCSystem` 合并 deformable surface 和 obstacle surface
- `CollisionCandidates` 已能生成 PT / EE 候选，并完成基础过滤
- `ObstacleBody` 已经是可用静态三角网格，不再是占位
- `IPCSystem` 已有明确的总能量装配入口：`compute_total_energy` / `compute_total_gradient` / `compute_total_hessian`

所以这部分现在不再是“先补前置、再设计”，而是可以直接进入实现。

## 当前代码现状下的真实边界

### 已具备，可直接复用

- 距离核接口已经稳定：

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

- `PointTriangleDistance::Result` 和 `EdgeEdgeDistance::Result` 都已经返回 12 DOF 局部梯度/Hessian，正好覆盖 barrier 链式法则需要的数据
- `CollisionMesh::vertices[i].global_vertex_index` 已经能区分 deformable 顶点和 obstacle 顶点
- `read_collision_vertex_position(mesh, state, vertex_idx)` 已经封装了“deformable 从 state 读、obstacle 从 static_position 读”
- `build_collision_candidates(mesh, state)` 已经能给出 PT / EE 候选

### 还没有，需要本 phase 新增

- 标量 barrier 数学函数
- 基于 PT / EE 候选的 pair barrier helper
- 局部 DOF 到全局 DOF 的装配 helper
- 接触 Hessian 的 PSD projection
- barrier 单元测试

### 还没有，但不阻塞本 phase

- CCD / collision-free step size
- `IPCSystem` 中把 barrier 接进总能量
- line search 的 `alpha_max`

也就是说，**Phase 2 现在应该独立做完并测通，再进入 Phase 4 / 5。**

## 本 phase 的目标

在不修改 `IPCSystem` 主循环的前提下，完成一个可独立测试的 barrier 模块：

1. 标量 barrier `b(s, s_hat)`
2. 对单个 PT / EE 候选计算 barrier energy / gradient / Hessian
3. 将局部贡献正确散装到全局 DOF
4. 保证 obstacle 侧不会写入全局梯度/Hessian
5. 为后续 `IPCSystem` 集成提供直接可调用的 `compute_total_*` 接口

## 建议落点

新增文件：

- `src/rtr/system/physics/ipc/contact/barrier_function.hpp`
- `src/rtr/system/physics/ipc/contact/barrier_potential.hpp`
- `test/system/physics/ipc/contact/barrier_function_test.cpp`
- `test/system/physics/ipc/contact/barrier_potential_test.cpp`

不建议把 barrier 放进 `energy/` 目录。原因很简单：它强依赖 `CollisionMesh + CollisionCandidates + Distance kernels`，语义上属于 contact，而不是像 inertial/material 那样的 body-local energy。

## 设计收敛

### 层 1: `barrier_function.hpp`

这是纯标量数学层，不涉及几何。

定义：

$$
b(s, \hat{s}) = \begin{cases}
-(s - \hat{s})^2 \ln(s / \hat{s}) & 0 < s < \hat{s} \\
0 & s \ge \hat{s}
\end{cases}
$$

其中：

- `s` 是距离平方
- `s_hat` 是 barrier 激活阈值，也使用距离平方

建议接口：

```cpp
namespace rtr::system::physics::ipc::barrier {

inline double energy(double s, double s_hat);
inline double gradient(double s, double s_hat);  // db/ds
inline double hessian(double s, double s_hat);   // d²b/ds²

}
```

导数公式沿用当前文档：

$$
\frac{\partial b}{\partial s}
= -(s - \hat{s})\left(2\ln(s/\hat{s}) + \frac{s - \hat{s}}{s}\right)
$$

$$
\frac{\partial^2 b}{\partial s^2}
= -2\ln(s/\hat{s}) - \frac{(s - \hat{s})(3s - \hat{s})}{s^2}
$$

### 数值约束

- `s_hat <= 0`：直接 `throw std::invalid_argument`
- `s >= s_hat`：返回 `0`
- `s <= 0`：不允许静默继续算 log，建议 `throw std::domain_error`

这里不要返回“极大值”。当前仓库的能量/几何模块风格都是输入非法时直接抛异常，barrier 也应保持一致。

## 层 2: `barrier_potential.hpp`

这一层负责把标量 barrier 和距离核结果拼起来，并对 `CollisionCandidates` 做全局装配。

### 建议配置

```cpp
struct BarrierPotentialConfig {
    double dhat_squared{0.01};
    double kappa{1e4};
    bool project_hessian_to_psd{true};
};
```

### 建议公开接口

```cpp
double compute_total_barrier_energy(
    const Eigen::VectorXd& x,
    const IPCState& state,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config
);

void accumulate_total_barrier_gradient(
    const Eigen::VectorXd& x,
    const IPCState& state,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    Eigen::VectorXd& gradient
);

void accumulate_total_barrier_hessian_triplets(
    const Eigen::VectorXd& x,
    const IPCState& state,
    const CollisionMesh& mesh,
    const CollisionCandidates& candidates,
    const BarrierPotentialConfig& config,
    std::vector<Eigen::Triplet<double>>& triplets
);
```

### 为什么这里保留 `state`

虽然 `x` 和当前 `state.x` 在大多数调用点会相同，但 barrier 在 Newton 过程中要对任意 trial `x` 评估。

当前代码里 obstacle 顶点位置存在 `mesh.vertices[i].static_position`，deformable 顶点读取路径则已经由 `read_collision_vertex_position(mesh, state, i)` 封装。不过这个 helper 固定读取 `state.x`，不适合 Newton trial `x`。

因此本 phase 应该在 `barrier_potential.hpp` 内新增一个局部 helper，语义如下：

```cpp
Eigen::Vector3d read_collision_vertex_position(
    const CollisionMesh& mesh,
    const IPCState& state,
    const Eigen::VectorXd& x,
    std::size_t surface_vertex_idx
);
```

约定：

- deformable 顶点从参数 `x` 读取
- obstacle 顶点仍从 `mesh.vertices[i].static_position` 读取
- `state` 只用于校验全局顶点索引范围和保持接口语义一致

不要直接复用现有 `collision_mesh.hpp` 里的同名函数，否则会把 barrier 错绑到 `state.x` 而不是 Newton 当前 `x`。

## 当前仓库下最自然的实现拆法

### Step A: 定义局部 pair 数据结构

建议在 `barrier_potential.hpp` 内部定义一个统一的局部装配载体：

```cpp
template <int Dofs>
struct PairBarrierContribution {
    double energy{0.0};
    Eigen::Matrix<double, Dofs, 1> gradient{Eigen::Matrix<double, Dofs, 1>::Zero()};
    Eigen::Matrix<double, Dofs, Dofs> hessian{Eigen::Matrix<double, Dofs, Dofs>::Zero()};
    std::array<std::optional<Eigen::Index>, Dofs> global_dof_map{};
};
```

这里 `Dofs` 对 PT / EE 都是 `12`，但保留模板更利于以后扩展。

### Step B: 实现泛型链式法则 helper

建议核心 helper：

```cpp
template <Distance DistanceKernel, int Dofs>
PairBarrierContribution<Dofs> compute_pair_barrier_from_distance_result(
    const typename DistanceKernel::Result& distance_result,
    const std::array<std::optional<Eigen::Index>, Dofs>& global_dof_map,
    const BarrierPotentialConfig& config
);
```

逻辑：

1. 读取 `s = distance_result.distance_squared`
2. 若 `s >= config.dhat_squared`，直接返回零贡献
3. 计算 `b(s)` / `b'(s)` / `b''(s)`
4. 链式法则：
   - `local_gradient = kappa * b'(s) * distance_gradient`
   - `local_hessian = kappa * (b''(s) * g g^T + b'(s) * H)`
5. `local_hessian = 0.5 * (H + H^T)` 对称化
6. 如启用 PSD 投影，则对局部 Hessian 做投影
7. 返回局部贡献

### Step C: PT / EE 的输入构造和 DOF map 构造

这一步不要再额外抽象过头，直接按候选类型写两个 helper 就够了。

#### PT helper

输入来源：

- 点顶点：`candidate.point_vertex_idx`
- 三角形：`mesh.triangles[candidate.triangle_idx]`

局部输入顺序必须严格对齐 `PointTriangleDistance::Input`：

```cpp
{ p, t0, t1, t2 }
```

对应的局部 12 个 DOF 顺序：

```text
0..2   -> p
3..5   -> t0
6..8   -> t1
9..11  -> t2
```

`global_dof_map` 的组装规则：

- deformable 顶点：写入连续 3 个全局 DOF 索引
- obstacle 顶点：3 个分量都写 `std::nullopt`

#### EE helper

输入来源：

- 边 A：`mesh.edges[candidate.edge_a_idx]`
- 边 B：`mesh.edges[candidate.edge_b_idx]`

局部输入顺序必须严格对齐 `EdgeEdgeDistance::Input`：

```cpp
{ ea0, ea1, eb0, eb1 }
```

对应的局部 12 个 DOF 顺序：

```text
0..2   -> ea0
3..5   -> ea1
6..8   -> eb0
9..11  -> eb1
```

### Step D: 全局散装 helper

建议单独做两个基础工具函数：

```cpp
template <int Dofs>
void scatter_pair_gradient(
    const Eigen::Matrix<double, Dofs, 1>& local_gradient,
    const std::array<std::optional<Eigen::Index>, Dofs>& global_dof_map,
    Eigen::VectorXd& global_gradient
);

template <int Dofs>
void scatter_pair_hessian_triplets(
    const Eigen::Matrix<double, Dofs, Dofs>& local_hessian,
    const std::array<std::optional<Eigen::Index>, Dofs>& global_dof_map,
    std::vector<Eigen::Triplet<double>>& triplets
);
```

规则：

- 行或列对应 `nullopt` 就跳过
- 只把 deformable 相关项写入全局
- 不需要在这里去重或合并 triplets，沿用当前 `MaterialEnergy` 风格，最后交给 `Eigen::SparseMatrix::setFromTriplets`

## PSD projection

### 为什么这一步要放在本 phase

虽然距离核本身返回 Hessian，但 barrier 组合后的局部 Hessian 不保证数值上对 Newton 友好。当前 solver 会正则化，但 barrier 部分最好自己先做局部 PSD 投影，减少不稳定源。

### 建议实现

局部矩阵维度只有 `12 x 12`，直接做稠密特征分解即可：

```cpp
Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 12, 12>> solver(H);
lambda = lambda.cwiseMax(0.0);
H_psd = Q * lambda.asDiagonal() * Q.transpose();
```

实现细节：

- 先对称化再分解
- 如特征分解失败，建议 `throw std::runtime_error`
- 只在 pair level 投影，不要等到全局 Hessian 再投影

## 推荐实现顺序

### Phase 2.1: 先做标量 barrier

交付：

- `barrier_function.hpp`
- `barrier_function_test.cpp`

完成标准：

- 标量 energy / gradient / hessian 全部可用
- FD check 通过
- 边界行为明确

### Phase 2.2: 做“局部 pair，不做全局装配”

交付：

- 一个内部 helper：给定 `Distance::Result + global_dof_map`，返回局部 energy/gradient/hessian
- PT / EE 的输入读取 helper

完成标准：

- 单对 PT gradient FD check 通过
- 单对 EE gradient FD check 通过
- Hessian 至少保证对称、有限

### Phase 2.3: 做全局散装

交付：

- `compute_total_barrier_energy`
- `accumulate_total_barrier_gradient`
- `accumulate_total_barrier_hessian_triplets`

完成标准：

- 多个候选对能正确累计
- obstacle 顶点不写入全局向量/矩阵
- 全局 triplets 索引都在合法范围内

## 与当前 `IPCSystem` 风格的对齐要求

当前 `IPCSystem` 不是通过一个统一 `Energy` 对象注册各项能量，而是在 `compute_total_energy` / `compute_total_gradient` / `compute_total_hessian` 中逐项手工累加。

因此 barrier 这里不要强行做成：

- `struct BarrierPotential : Energy`
- 或依赖某个全新 runtime object

应该直接提供和当前系统兼容的自由函数，让 Phase 5 可以这样接：

```cpp
total_energy += compute_total_barrier_energy(x, state, mesh, candidates, config);
accumulate_total_barrier_gradient(x, state, mesh, candidates, config, gradient);
accumulate_total_barrier_hessian_triplets(x, state, mesh, candidates, config, triplets);
```

## 测试细化

### `barrier_function_test.cpp`

至少覆盖：

- `0 < s < s_hat` 区间内 energy 有限且非负
- gradient 对 energy 做 FD check
- hessian 对 gradient 做 FD check
- `s == s_hat` 时 `energy == 0`
- `s > s_hat` 时三者均为 `0`
- `s <= 0` / `s_hat <= 0` 抛异常

### `barrier_potential_test.cpp`

建议拆成 5 组：

1. `compute_pair_barrier<PointTriangleDistance>` gradient FD check
2. `compute_pair_barrier<EdgeEdgeDistance>` gradient FD check
3. obstacle 顶点对应的 `nullopt` 不写入全局 gradient
4. `accumulate_total_barrier_hessian_triplets` 产生的 triplets 都在合法 DOF 范围内
5. 当 `distance_squared >= dhat_squared` 时 barrier 贡献为零

### 测试输入建议

优先复用已有几何测试的构型风格，避免新造非常极端的退化数据：

- PT：点在三角形上方，距离小于 `sqrt(dhat_squared)`
- EE：两条错位且非平行的边，距离小于 `sqrt(dhat_squared)`

不要拿严格退化三角形/平行边作为 barrier 主测试输入，这些情况应该继续由距离核自己的测试负责。

## 实现时需要避免的两个坑

### 坑 1: 误用 `state.x` 代替当前 `x`

这是当前最容易犯的错误。Barrier 在 Newton 里评估的是 trial configuration，不是系统缓存的 `state.x`。

### 坑 2: `global_vertex_index` 不是 DOF 起始索引

`CollisionVertex::global_vertex_index` 语义是“全局顶点索引”，不是“全局 DOF 索引”。真正 DOF 基址应为：

```cpp
3 * (*global_vertex_index)
```

文档后续提到的 `global_dof_map`，映射的是每个局部分量到全局 DOF 分量，而不是顶点到顶点。

## 参数建议

| 参数 | 当前阶段推荐值 | 说明 |
|------|----------------|------|
| `dhat_squared` | `1e-2` | 对应 `dhat ≈ 0.1`，适合当前 block/plane 量级 |
| `kappa` | `1e3` 起步 | 先保证数值稳定，再往上调刚度 |
| `project_hessian_to_psd` | `true` | Day 3 默认打开 |

## 完成定义

当下面几件事都成立时，这个 phase 就算完成，可以进入 Phase 4 / 5：

- barrier 数学函数和导数实现完毕
- PT / EE pair barrier helper 完成
- 全局 gradient / Hessian triplets 装配完成
- 单元测试通过
- 接口已经可以被 `IPCSystem` 直接调用

## 输出

- `src/rtr/system/physics/ipc/contact/barrier_function.hpp`
- `src/rtr/system/physics/ipc/contact/barrier_potential.hpp`
- `test/system/physics/ipc/contact/barrier_function_test.cpp`
- `test/system/physics/ipc/contact/barrier_potential_test.cpp`
