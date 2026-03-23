# `ipc_system.hpp`

`src/rtr/system/physics/ipc/core/ipc_system.hpp` 是 IPC 可变形体仿真的顶层编排器。它将数据结构、能量模块和 Newton 求解器串联成一个统一的 `step(delta_seconds)` 接口。

## 理论基础：Backward Euler 作为优化问题

### 从牛顿第二定律到优化问题

连续运动方程为 $M\ddot{x} = f(x)$，其中 $f$ 包括内部弹性力和外力（重力）。用 backward Euler 离散化：

$$
M \frac{x^{n+1} - 2x^n + x^{n-1}}{h^2} = f(x^{n+1})
$$

整理：

$$
M(x^{n+1} - \hat{x}) = h^2 f(x^{n+1})
$$

其中 $\hat{x} = 2x^n - x^{n-1} = x^n + h v^n$ 是**惯性预测位置**（如果没有力作用，物体会到达的位置）。

**关键洞察**：如果所有力都来自势能（$f = -\nabla \Psi$），这个方程就是以下优化问题的驻点条件：

$$
\boxed{x^{n+1} = \arg\min_x \underbrace{\frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})}_{E_{\text{inertial}}} + \underbrace{\Psi_{\text{elastic}}(x)}_{E_{\text{elastic}}} + \underbrace{(-f_g^T x)}_{E_{\text{gravity}}} }
$$

**证明**：令总能量的梯度为零：

$$
\nabla E = \frac{M}{h^2}(x - \hat{x}) + \nabla\Psi - f_g = 0
$$

$$
M(x - \hat{x}) = h^2(-\nabla\Psi + f_g) = h^2 f(x)
$$

这正是 backward Euler 方程。

### 为什么用优化

| 方法 | 优点 | 缺点 |
|------|------|------|
| 直接求解 $Ax = b$ | 简单 | 只适用于线性系统 |
| 不动点迭代 | 容易实现 | 刚性问题收敛慢 |
| **优化** | 处理非线性，保证能量下降 | 需要 energy/gradient/Hessian |

优化表述的优势：
1. **天然处理非线性弹性**——不需要线性化近似
2. **能量下降由 line search 保证**——无条件稳定
3. **干净地扩展到 IPC**——barrier 能量只是总和中多一项

## 架构：各模块如何串联

```
┌─────────────────────────────────────────────────────────┐
│                       IPCSystem                          │
│                                                          │
│  ┌──────────┐   ┌─────────────────────────────────────┐ │
│  │ IPCConfig │   │           IPCState                  │ │
│  │  dt       │   │  x (3n)  x_prev  v  mass_diag      │ │
│  │  gravity  │   └─────────────────────────────────────┘ │
│  │  solver   │                                           │
│  │  params   │   ┌──────────┐  ┌──────────┐             │
│  └──────────┘   │ TetBody 0 │  │ TetBody 1 │  ...       │
│                  │ geometry  │  │ geometry  │             │
│                  │ material  │  │ material  │             │
│                  │ masses    │  │ masses    │             │
│                  │ fixed_v   │  │ fixed_v   │             │
│                  └──────────┘  └──────────┘             │
│                                                          │
│  ┌──────────────── 总能量 ──────────────────────────┐   │
│  │                                                    │   │
│  │  InertialEnergy  +  GravityEnergy  +  Σ Material  │   │
│  │  （二次）          （线性）          （非线性）    │   │
│  │                                                    │   │
│  │  每项提供：energy / gradient / hessian_triplets     │   │
│  └──────────────────────────────────────────────────┘   │
│                         │                                │
│                         ▼                                │
│               ┌─────────────────┐                        │
│               │  Newton Solver   │                        │
│               │  H·Δx = -g      │                        │
│               │  + line search   │                        │
│               │  + DBC mask      │                        │
│               └─────────────────┘                        │
│                         │                                │
│                         ▼                                │
│               state.x 更新                               │
│               state.v = (x - x_prev) / dt               │
└─────────────────────────────────────────────────────────┘
```

### 各层职责

| 层 | 文件 | 职责 |
|----|------|------|
| **数据** | `ipc_state.hpp`, `tet_body.hpp` | 全局 DOF 向量，per-body 的几何/材料/约束 |
| **能量** | `inertial_energy.hpp`, `gravity_energy.hpp`, `material_energy.hpp` | 各项 energy/gradient/Hessian 计算 |
| **材料** | `tet_fixed_corotated.hpp`, `tet_material_variant.hpp` | 本构模型（$F \to \Psi, P, \partial^2\Psi$） |
| **求解器** | `newton_solver.hpp`, `line_search.hpp` | 带 DBC 和 line search 的非线性求解 |
| **系统** | `ipc_system.hpp` | 编排：装配、求解、更新 |

## `IPCConfig`

```cpp
struct IPCConfig {
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};     // 重力加速度
    NewtonSolverParams solver_params{};             // Newton 迭代控制
};
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `gravity` | $(0, -9.81, 0)$ | 标准地球重力。零重力测试时设为零。 |
| `solver_params` | 见 Newton 文档 | `max_iterations=50`，`gradient_tolerance=1e-6` 等。 |

## Runtime 组装与 `initialize()`

### 做了什么

```cpp
void rebuild_runtime_state() {
    // 阶段 1：按 IPCBodyID 保留旧的 per-body 状态
    // 阶段 2：Per-body 预计算和 DOF 重新分配
    for (auto body_id : m_body_order) {
        auto& body = m_tet_bodies.at(body_id);
        body.precompute();           // Dm_inv, rest_volumes, vertex_masses
        body.info.dof_offset = dof_offset;
        body.info.vertex_count = body.vertex_count();
        total_vertices += body.vertex_count();
        dof_offset += 3u * body.vertex_count();
    }

    // 阶段 3：全局状态分配
    m_state.resize(total_vertices);  // 分配 x, x_prev, v, mass_diag（都是 3n）

    // 阶段 4：如果 IPCBodyID 仍然存在就恢复旧状态，
    //          否则从 rest positions 初始化
    for (auto body_id : m_body_order) {
        const auto& body = m_tet_bodies.at(body_id);
    }

    // 阶段 5：构建 DBC mask
    m_x_hat = m_state.x_prev;
    build_free_dof_mask();
    m_initialized = true;
}
```

`initialize()` 现在只是显式触发这次重建的入口。`create_tet_body` / `remove_tet_body` 会把系统标记为 dirty，而 `step(delta_seconds)` 会在求解前自动重建。

`body.precompute()` 会从每个 body 的 material object 里读取 density，因此 `mass_diag` 的来源是 `TetBody::material`。

### DOF Offset 布局

有 $B$ 个 body，各有 $n_0, n_1, \ldots, n_{B-1}$ 个顶点：

```
全局 DOF 向量 x（大小 3N，N = Σn_b）：

┌─── body 0 ────┐┌─── body 1 ────┐┌─── body 2 ────┐
[x0 y0 z0 x1 y1 z1 ...][x0 y0 z0 ...][x0 y0 z0 ...]
 ↑                       ↑              ↑
 dof_offset=0            dof_offset     dof_offset
                         =3*n_0         =3*(n_0+n_1)
```

`body.info.dof_offset` 告诉该 body 的 DOF 在全局向量中从哪里开始。所有能量模块用它来索引 $x$。

### `build_free_dof_mask()`

```cpp
void build_free_dof_mask() {
    m_free_dof_mask.assign(m_state.dof_count(), true);  // 默认全部自由
    for (const auto& body : m_tet_bodies) {
        for 每个 fixed vertex:
            m_free_dof_mask[3*global_vertex + 0] = false;  // x
            m_free_dof_mask[3*global_vertex + 1] = false;  // y
            m_free_dof_mask[3*global_vertex + 2] = false;  // z
    }
}
```

将 per-vertex 约束展开为 per-DOF 布尔值。Newton solver 用这个 mask 从线性系统中消去约束 DOF（见 `newton_solver.hpp` 文档）。

## 时间步进：`step(delta_seconds)`

### 完整流程

```cpp
void step(double delta_seconds) {
    // 1. 保存上一步状态
    m_state.x_prev = m_state.x;

    // 2. 惯性预测
    compute_x_hat(delta_seconds);   // x_hat = x_prev + dt * v

    // 3. 组装 Newton 回调
    NewtonProblem problem{
        .compute_energy = [this, delta_seconds](x) { return compute_total_energy(x, delta_seconds); },
        .compute_gradient = [this, delta_seconds](x, g) { compute_total_gradient(x, g, delta_seconds); },
        .compute_hessian_triplets =
            [this, delta_seconds](x, t) { compute_total_hessian(x, t, delta_seconds); },
    };

    // 4. Newton 求解（原地修改 m_state.x）
    solve(m_state, m_x_hat, m_free_dof_mask, problem, m_config.solver_params);

    // 5. 更新速度
    m_state.v = (m_state.x - m_state.x_prev) / delta_seconds;

    // 6. 约束 DOF 速度清零
    for 每个 DOF i:
        if (!m_free_dof_mask[i]):
            m_state.v[i] = 0.0;
}
```

### 逐阶段理论

#### 阶段 1：保存上一步状态

$x^{n-1} \leftarrow x^n$

速度计算和惯性能量项都需要。

#### 阶段 2：惯性预测

$$
\hat{x} = x^n + h \cdot v^n
$$

```cpp
void compute_x_hat(double delta_seconds) {
    m_x_hat = m_state.x_prev + delta_seconds * m_state.v;
}
```

$\hat{x}$ 是没有力作用时系统会到达的位置。它是上一步动量的"延续"。重力和弹性力会在 Newton 求解中把 $x^{n+1}$ 从 $\hat{x}$ 拉开。

**为什么不把重力放入 $\hat{x}$**：见 `gravity_energy.hpp` 文档。保持 $\hat{x}$ 为纯运动学使能量分解更干净，每一项可以独立调试。

#### 阶段 3：Newton Problem 组装

`NewtonProblem` 是一个回调包，solver 用它来评估总能量面：

```cpp
const NewtonProblem problem{
    .compute_energy = [this](const Eigen::VectorXd& x) {
        return compute_total_energy(x);
    },
    .compute_gradient = [this](const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
        compute_total_gradient(x, gradient);
    },
    .compute_hessian_triplets = [this](const Eigen::VectorXd& x,
                                       std::vector<Eigen::Triplet<double>>& triplets) {
        compute_total_hessian(x, triplets);
    },
};
```

Lambda 捕获 `this`，使 solver 能回调 IPCSystem 的能量装配，而不需要知道具体的能量项。这种解耦意味着新增能量项时 solver 代码不需要改。

#### 阶段 4：Newton 求解

```cpp
const NewtonSolverResult result =
    solve(m_state, m_x_hat, m_free_dof_mask, problem, m_config.solver_params);
```

Solver 通过迭代 Newton 步原地修改 `m_state.x`（见 `newton_solver.hpp` 文档）。返回时，`m_state.x` 包含 $x^{n+1}$。

如果求解未收敛，记录警告但仿真继续运行——这在困难配置下很常见，通常在后续步中会自我修正。

#### 阶段 5-6：速度更新

$$
v^{n+1} = \frac{x^{n+1} - x^n}{h}
$$

```cpp
m_state.v = (m_state.x - m_state.x_prev) / delta_seconds;
```

然后对约束 DOF 的速度清零：

```cpp
for (Eigen::Index i = 0; i < m_state.v.size(); ++i) {
    if (!m_free_dof_mask[static_cast<std::size_t>(i)]) {
        m_state.v[i] = 0.0;
    }
}
```

**为什么要清零约束速度**：否则固定顶点会因 $x^{n+1} - x^n$ 的浮点噪声而积累虚假速度，导致后续步中 $\hat{x}$ 漂移。

## 总能量装配

### 能量

$$
E(x) = E_{\text{inertial}}(x) + E_{\text{gravity}}(x) + \sum_{b} E_{\text{material}}^{(b)}(x)
$$

```cpp
double compute_total_energy(const Eigen::VectorXd& x, double delta_seconds) const {
    double total_energy = 0.0;

    // 惯性：(1/2h²) (x - x_hat)^T M (x - x_hat)
    total_energy += InertialEnergy::compute_energy(InertialEnergy::Input{
        .x = x, .x_hat = m_x_hat, .mass_diag = m_state.mass_diag, .dt = delta_seconds,
    });

    // 重力：-f_g^T x = -Σ m_i * g * x_i
    total_energy += GravityEnergy::compute_energy(GravityEnergy::Input{
        .x = x, .mass_diag = m_state.mass_diag, .gravity = m_config.gravity,
    });

    // 弹性：Σ_body Σ_tet V_e * Ψ(F)
    for (const auto& body : m_tet_bodies) {
        total_energy += material_energy_variant::compute_energy(body, x);
    }

    return total_energy;
}
```

每个能量项使用 `Energy` concept 的 `Input` struct 模式——装配代码只需构造输入并调用静态方法。

### 梯度

$$
\nabla E = \frac{M}{h^2}(x - \hat{x}) - f_g + \sum_b \nabla E_{\text{material}}^{(b)}
$$

```cpp
void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient) const {
    gradient.setZero();
    InertialEnergy::compute_gradient({...}, gradient);   // += M/h² (x - x_hat)
    GravityEnergy::compute_gradient({...}, gradient);    // += -m*g
    for (const auto& body : m_tet_bodies) {
        material_energy_variant::compute_gradient(body, x, gradient);  // += elastic
    }
}
```

所有梯度方法使用 `+=` 累加模式——`gradient` 清零一次，然后每项加入贡献。避免分配临时向量。

### Hessian

$$
H = \frac{M}{h^2} + \sum_b H_{\text{material}}^{(b)}
$$

```cpp
void compute_total_hessian(const Eigen::VectorXd& x,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    triplets.clear();
    InertialEnergy::compute_hessian_triplets({...}, triplets);   // 对角 M/h²
    // GravityEnergy：零 Hessian（线性能量），无贡献
    for (const auto& body : m_tet_bodies) {
        material_energy_variant::compute_hessian_triplets(body, x, triplets);  // per-tet 12×12
    }
}
```

注意：重力不贡献 Hessian（线性能量 → 常数梯度 → 零 Hessian）。这就是为什么 `GravityEnergy::compute_hessian_triplets` 是 no-op。

### Variant Dispatch

每个 body 携带自己的 `TetMaterialVariant material` 字段。`material_energy_variant` 函数用 `std::visit` 做 dispatch：

```cpp
inline double compute_energy(const TetBody& body, const Eigen::VectorXd& x) {
    return std::visit([&](const auto& material) {
        return MaterialEnergy<std::decay_t<decltype(material)>>::compute_energy(
            {.body = body, .x = x, .material = material});
    }, body.material);
}
```

Lambda 的 `const auto&` 参数意味着编译器为 variant 的每个替代类型生成一份特化。内层 tet 循环作为完全内联的模板代码运行——没有逐 tet 的虚函数 dispatch。

## Hessian 稀疏结构

装配后的 Hessian 有特定的稀疏模式：

```
  ┌─────────────────────────────┐
  │ M/h²（对角）                │  ← InertialEnergy
  │  +                          │
  │ ┌───┐                       │
  │ │12×12│    （tet 0）        │  ← MaterialEnergy
  │ └───┘                       │
  │       ┌───┐                 │
  │       │12×12│（tet 1）      │
  │       └───┘                 │
  │             ⋱               │
  │                  ┌───┐      │
  │                  │12×12│    │
  │                  └───┘      │
  └─────────────────────────────┘
```

- **惯性**：$3N$ 个对角元素 $m_i / h^2$
- **弹性**：per-tet $12 \times 12$ block（4 顶点 × 3 DOF），共享顶点处重叠

不同项的 triplets 在 Newton solver 中通过带 sum functor 的 `setFromTriplets` 累加。

## 状态访问

```cpp
const IPCState& state() const;                              // 完整全局状态
bool has_tet_body(IPCBodyID id) const;
const TetBody& get_tet_body(IPCBodyID id) const;            // 按 stable ID 取 body
std::vector<Eigen::Vector3d> get_body_positions(IPCBodyID body_id) const;
```

`get_body_positions` 从全局 DOF 向量提取一个 body 的当前顶点位置：

```cpp
std::vector<Eigen::Vector3d> get_body_positions(IPCBodyID body_id) const {
    const auto& body = m_tet_bodies.at(body_id);
    std::vector<Eigen::Vector3d> positions{};
    positions.reserve(body.vertex_count());
    const std::size_t base_vertex = body.info.dof_offset / 3u;
    for (std::size_t local_vertex = 0; local_vertex < body.vertex_count(); ++local_vertex) {
        positions.push_back(m_state.position(base_vertex + local_vertex));
    }
    return positions;
}
```

Scene sync 层用它把变形后的位置写回 `DeformableMeshComponent` 用于渲染。

## 完整数据流：一个时间步

```
                    IPCSystem::step()
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
     x_prev = x     compute_x_hat()   构建回调
                    x_hat = x + h·v    NewtonProblem{E, g, H}
                          │
                          ▼
                    Newton Solver
                    ┌─────────────────────────────────────┐
                    │ for k = 0 to max_iter:              │
                    │   g ← ∇E(x)     ┌────────────────┐ │
                    │                  │ InertialEnergy  │ │
                    │                  │ GravityEnergy   │ │
                    │   H ← ∇²E(x)   │ MaterialEnergy  │ │
                    │                  │  per body       │ │
                    │                  │   per tet       │ │
                    │                  │    F = Ds·Dm⁻¹  │ │
                    │                  │    Ψ, P, ∂²Ψ   │ │
                    │                  └────────────────┘ │
                    │   H_FF · Δx_F = -g_F （reduced）   │
                    │   Δx ← expand(Δx_F)                │
                    │   α ← line_search(E, x, Δx)        │
                    │   x ← x + α·Δx                     │
                    └─────────────────────────────────────┘
                          │
                          ▼
                    v = (x - x_prev) / h
                    约束 DOF 速度清零
                          │
                          ▼
                    state.x, state.v 就绪
                          │
                          ▼
                    （scene sync 读取 state.x
                     → tet_to_mesh → DeformableMeshComponent
                     → GPU 上传 → 渲染）
```

## 数值示例：单 Tet 自由落体

设置：1 个 tet（4 个顶点），$h = 0.01$，$g = (0, -9.81, 0)$，$E = 10^5$，$\nu = 0.3$，无 DBC。

**第 0 步（initialize）**：
- `x = x_prev = rest_positions`（全部在 rest 位置）
- `v = 0`
- `mass_diag`：每个顶点得到 $\rho V_e / 4$，重复 3 次

**第 1 步**：
- `x_hat = x + 0 = x`（还没有速度）
- Newton 第 1 次迭代：
  - $E_{\text{inertial}} = 0$（x = x_hat）
  - $E_{\text{gravity}} = -\sum m_i g_y y_i < 0$（低于参考位置）
  - $\nabla E_{\text{gravity}} = (0, +m \cdot 9.81, 0)$（每个顶点，向上）
  - $\nabla E_{\text{inertial}} = 0$
  - Newton 步将顶点向下推（梯度的反方向）
  - Line search 后：顶点下移约 $\frac{1}{2} g h^2 \approx 4.9 \times 10^{-4}$ m
- 速度更新：$v_y \approx -9.81 \times 0.01 = -0.0981$ m/s

**第 2 步**：
- `x_hat = x + 0.01 * v`（包含第 1 步的动量）
- 重力继续加速：$v_y \approx -0.1962$ m/s
- Tet 因惯性 vs 弹性恢复力而略微拉伸

## 错误处理

| 情况 | 响应 |
|------|------|
| 没有显式 `initialize()` 就先 `step()` | 若存在 body，会先自动重建 |
| body 列表为空 | `step()` 直接返回（no-op） |
| Newton 未收敛 | 警告日志，仿真继续 |
| `get_body_positions` 在 init 之前调用 | `throw std::logic_error` |
| `IPCBodyID` 不存在 | `throw std::out_of_range`（来自 `.at()`） |

## 扩展点

| 扩展 | 修改位置 |
|------|----------|
| 新能量项（barrier、friction） | 加入 `compute_total_energy/gradient/hessian` |
| 新材料模型 | 加入 `TetMaterialVariant`，实现 `TetMaterialModel` concept |
| Per-axis DBC（slip） | 修改 `build_free_dof_mask` 从 `AxisConstraint` 而非 `fixed_vertices` 读取 |
| CCD 步长裁剪 | 在 Newton 回调中传 `alpha_max` 给 line search |
| 多体碰撞 | 在 Newton 前添加 contact pair 检测，加 barrier 能量项 |
