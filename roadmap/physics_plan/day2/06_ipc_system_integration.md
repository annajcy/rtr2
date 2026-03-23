# Phase 6: 接入 IPCSystem

## 目标

把 Phase 1-5 实现的 geometry、barrier、CCD、obstacle 模块接入 `IPCSystem` 的主循环，使 `step()` 完成带接触的完整时间步。

## 当前 IPCSystem.step() 流程

```text
1. rebuild_runtime_state()（如果需要）
2. x_prev = x
3. compute_x_hat()
4. Newton solver:
   a. compute_total_energy(x)     = E_inertial + E_gravity + Ψ_elastic
   b. compute_total_gradient(x)
   c. compute_total_hessian(x)
   d. solve H·dx = -g（reduced system）
   e. backtracking line search
   f. x += alpha * dx
5. v = (x - x_prev) / dt
```

## 修改后的 step() 流程

```text
1. rebuild_runtime_state()（如果需要）
2. x_prev = x
3. compute_x_hat()
4. build collision mesh（合并 tet 表面 + obstacle 表面）       ← 新增
5. Newton solver:
   a. update collision candidates（broad phase + 过滤）        ← 新增
   b. compute_total_energy(x)     = E_inertial + E_gravity + Ψ_elastic + κ·B(x)  ← 修改
   c. compute_total_gradient(x)                                ← 修改
   d. compute_total_hessian(x)                                 ← 修改
   e. solve H·dx = -g（reduced system）
   f. compute collision-free stepsize alpha_ccd                ← 新增
   g. backtracking line search (alpha_init = min(1, alpha_ccd)) ← 修改
   h. x += alpha * dx
6. v = (x - x_prev) / dt
```

### 关键变化

1. **Collision mesh 在每个 time step 开始时构建一次**（拓扑不变，只需构建一次）
2. **Candidates 在每次 Newton 迭代开始时更新**（因为 x 变了，距离变了，候选对可能变）
3. **总能量/梯度/Hessian 新增 barrier 贡献**
4. **Line search 受 CCD 安全步长约束**

---

## IPCConfig 扩展

```cpp
struct IPCConfig {
    double dt{0.01};
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};

    // 新增：接触参数
    double dhat_squared{0.01};      // barrier 激活距离阈值（距离平方）
    double barrier_stiffness{1e4};  // κ
    bool enable_contact{true};      // 方便关闭接触做对比
};
```

## IPCSystem 新增成员

```cpp
// 新增私有成员
std::unordered_map<IPCBodyID, ObstacleBody> m_obstacle_bodies{};
CollisionMesh m_collision_mesh{};
CollisionCandidates m_candidates{};
```

## 能量装配修改

```cpp
double compute_total_energy(const Eigen::VectorXd& x) const {
    double total_energy = 0.0;
    total_energy += InertialEnergy::compute_energy(...);
    total_energy += GravityEnergy::compute_energy(...);
    for (const auto& [id, body] : m_tet_bodies) {
        total_energy += material_energy_variant::compute_energy(body, x);
    }

    // 新增：barrier energy
    if (m_config.enable_contact && !m_candidates.empty()) {
        total_energy += compute_barrier_energy(
            x, m_collision_mesh, m_candidates,
            {.dhat_squared = m_config.dhat_squared,
             .kappa = m_config.barrier_stiffness}
        );
    }

    return total_energy;
}
```

gradient 和 Hessian 同理。

## Newton solver 修改

当前 `newton_solver.hpp` 的 `solve()` 函数通过 `NewtonProblem` 回调来获取 energy/gradient/Hessian。**不需要修改 solver 本身**——只需要在回调中加入 barrier 贡献。

但 line search 需要修改：加入 `alpha_max` 参数。

### line_search.hpp 修改

```cpp
LineSearchResult backtracking_line_search(
    // ... existing params ...
    double alpha_max = 1.0  // 新增：CCD 安全步长上界
);
```

初始 alpha 从 `min(1.0, alpha_max)` 开始回溯。

### newton_solver.hpp 修改

`NewtonProblem` 新增可选的 CCD 回调：

```cpp
struct NewtonProblem {
    std::function<double(const Eigen::VectorXd&)> compute_energy;
    std::function<void(const Eigen::VectorXd&, Eigen::VectorXd&)> compute_gradient;
    std::function<void(const Eigen::VectorXd&, std::vector<Eigen::Triplet<double>>&)> compute_hessian_triplets;

    // 新增：可选，CCD 安全步长回调
    std::function<double(const Eigen::VectorXd& x, const Eigen::VectorXd& dx)> compute_max_stepsize{};
};
```

如果 `compute_max_stepsize` 非空，solver 在 line search 前调用它获取 `alpha_max`。

### Newton 迭代中更新 candidates

如果 Day 2 时间足够，可以在每次 Newton 迭代开始时更新 candidates：

```cpp
// 在 Newton 迭代开始时（energy assembly 前）
if (config.enable_contact) {
    m_candidates = generate_collision_candidates(m_collision_mesh, m_state.x);
}
```

如果时间不够，也可以简化为整个 time step 只做一次 candidate generation（在 collision mesh 构建后）。这会漏掉一些在 Newton 迭代过程中新出现的接近接触对，但对 Day 2 demo 够用。

---

## Obstacle API

```cpp
IPCBodyID create_obstacle_body(ObstacleBody body) {
    const IPCBodyID body_id = m_next_body_id++;
    m_obstacle_bodies.emplace(body_id, std::move(body));
    m_needs_rebuild = true;
    return body_id;
}
```

注意：obstacle body 加入/删除也要触发 collision mesh 重建，但不触发 DOF 重建。

---

## 输出文件

修改已有文件：

| 文件 | 修改内容 |
|------|---------|
| `ipc_system.hpp` | 新增 obstacle 管理、barrier 装配、CCD 集成 |
| `newton_solver.hpp` | `NewtonProblem` 新增 `compute_max_stepsize`；solver 集成 CCD |
| `line_search.hpp` | 新增 `alpha_max` 参数 |

## 测试

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/ipc_contact_smoke_test.cpp` | tet block 落到地面上不穿透 |
