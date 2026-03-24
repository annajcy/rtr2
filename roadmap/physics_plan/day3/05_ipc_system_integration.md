# Phase 5: IPCSystem Contact 集成

## 目标

把 Phase 1-4 的所有模块接入 `IPCSystem` 的主循环，使 `step()` 完成带接触的完整时间步。

这是 Day 3 风险最高的 Phase —— 第一次真正修改主循环。

## 修改后的 step() 流程

```text
1. rebuild_runtime_state()（如果需要）
2. x_prev = x
3. compute_x_hat()
4. build collision mesh（合并 tet 表面 + obstacle 表面）       ← 新增
5. Newton solver:
   a. update collision candidates（broad phase + 过滤）        ← 新增
   b. compute_total_energy(x) = E_inertial + E_gravity + Ψ_elastic + κ·B(x)  ← 修改
   c. compute_total_gradient(x)                                ← 修改
   d. compute_total_hessian(x)                                 ← 修改
   e. solve H·dx = -g（reduced system）
   f. compute collision-free stepsize alpha_ccd                ← 新增
   g. backtracking line search (alpha ≤ alpha_ccd)            ← 修改
   h. x += alpha * dx
6. v = (x - x_prev) / dt
```

## IPCConfig 扩展

```cpp
struct IPCConfig {
    double dt{0.01};
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};

    // 新增
    double dhat_squared{0.01};
    double barrier_stiffness{1e4};
    bool enable_contact{true};
};
```

## IPCSystem 新增成员

```cpp
// 私有成员
std::unordered_map<IPCBodyID, ObstacleBody> m_obstacle_bodies{};
CollisionMesh m_collision_mesh{};
CollisionCandidates m_candidates{};
```

## 三个精确插入点

### 插入点 A: 总能量装配

```cpp
double compute_total_energy(const Eigen::VectorXd& x) const {
    double total = /* existing inertial + gravity + elastic */;

    if (m_config.enable_contact && !m_candidates.empty()) {
        total += compute_total_barrier_energy(
            x, m_collision_mesh, m_candidates,
            {.dhat_squared = m_config.dhat_squared,
             .kappa = m_config.barrier_stiffness}
        );
    }
    return total;
}
```

gradient 和 Hessian 同理，调用 `accumulate_total_barrier_gradient` / `accumulate_total_barrier_hessian_triplets`。

### 插入点 B: NewtonProblem.compute_max_stepsize

```cpp
problem.compute_max_stepsize = [&](const Eigen::VectorXd& x,
                                    const Eigen::VectorXd& dx) -> double {
    if (!m_config.enable_contact) return 1.0;
    return compute_collision_free_stepsize(x, dx, m_collision_mesh, m_candidates, ccd_config);
};
```

### 插入点 C: Newton 迭代中更新 candidates

简化版（整个 time step 一次 candidate generation）：

```cpp
// step() 中 collision mesh 构建后
if (m_config.enable_contact) {
    m_collision_mesh = build_collision_mesh(m_tet_bodies, m_obstacle_bodies, m_state);
    m_candidates = generate_collision_candidates(m_collision_mesh, m_state.x);
}
```

完善版（每次 Newton 迭代更新 candidates）：在 `NewtonProblem` 的回调中每次重新生成。Day 3 先用简化版，如果 demo 稳定就不改。

## Obstacle API

```cpp
IPCBodyID create_obstacle_body(ObstacleBody body);
bool has_obstacle_body(IPCBodyID id) const;
bool remove_obstacle_body(IPCBodyID id);
```

Obstacle body 添加/删除触发 `m_needs_rebuild = true`，但 `rebuild_runtime_state()` 中不重建 DOF（obstacle 不在 DOF 中），只重建 collision mesh。

## 关键安全措施

### `enable_contact = false` 回退

所有 contact 相关代码都必须被 `enable_contact` 守卫。当 `enable_contact = false` 时，行为与 Day 1 完全一致。

### contact 不破坏无接触路径

- 无 obstacle body + `enable_contact = true` → 不崩溃，candidates 为空，barrier 贡献为零
- 有 obstacle body + `enable_contact = false` → obstacle 被忽略

## 测试

### `ipc_contact_smoke_test.cpp`

1. tet block 从 y=1.5 落到 y=0 的 ground plane，100 步后 `y_min > -0.01`
2. 同样场景 `enable_contact = false`，block 穿过地面（验证 barrier 确实生效）
3. 多步运行无 NaN
4. barrier energy > 0 when distance < dhat

## 输出

修改：
- `src/rtr/system/physics/ipc/core/ipc_system.hpp`

新增测试：
- `test/system/physics/ipc/ipc_contact_smoke_test.cpp`
