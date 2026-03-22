# Phase 4: IPCSystem 组装

## File: `core/ipc_system.hpp`

Day 1 的 IPCSystem 是一个独立的 simulation driver，不接入 PhysicsSystem 主循环。

### 接口

```cpp
namespace rtr::system::physics::ipc {

struct IPCConfig {
    double dt{0.01};                      // 固定时间步
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};
};

class IPCSystem {
public:
    explicit IPCSystem(IPCConfig config);

    // Body 注册
    void add_tet_body(TetBody body);
    // void add_obstacle_body(ObstacleBody body);  // Day 3

    // 初始化：组装全局状态、预计算
    void initialize();

    // 单步推进
    void step();

    // 状态访问
    const IPCState& state() const;
    const TetBody& tet_body(std::size_t index) const;

    // 顶点回读（用于 scene write-back）
    std::vector<Eigen::Vector3d> get_body_positions(std::size_t body_index) const;

private:
    IPCConfig m_config;
    IPCState m_state;
    std::vector<TetBody> m_tet_bodies;
    // std::vector<ObstacleBody> m_obstacle_bodies;  // Day 3

    FixedCorotatedMaterial m_material;  // concept，编译期多态，无虚函数开销

    Eigen::VectorXd m_x_hat;  // 预测位置
    std::vector<bool> m_free_dof_mask;  // per-DOF, 从 vertex_constraints 展开

    void compute_x_hat();
    void build_free_dof_mask();

    // 总能量装配
    double compute_total_energy(const Eigen::VectorXd& x);
    void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient);
    void compute_total_hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets);
};

}
```

### initialize() 做的事

1. 对每个 TetBody 调 `precompute()`
2. 计算全局顶点总数，resize `IPCState`
3. 把每个 body 的 rest positions 写入 `state.x` 和 `state.x_prev`
4. 把每个 body 的质量写入 `state.mass_diag`
5. 速度初始化为 0
6. 构建 `m_free_dof_mask`

### step() 做的事

```
1. state.x_prev = state.x
2. compute_x_hat()
3. newton_solve(state, x_hat, free_dof_mask, ...)
4. state.v = (state.x - state.x_prev) / dt
5. 对 fixed DOFs: state.v[i] = 0  （按 free_dof_mask 逐 DOF 清零）
   // stick vertex: 3 个分量全清零
   // slip vertex: 只清零被约束的轴分量，保留自由轴的速度
```

### compute_x_hat()

$\hat{x}$ 是纯惯性预测位置，不含外力：

```cpp
// x_hat = x_prev + dt * v  （纯惯性，重力由 GravityEnergy 独立处理）
m_x_hat = m_state.x_prev + m_config.dt * m_state.v;
```

### 总能量装配

```cpp
double compute_total_energy(const Eigen::VectorXd& x) {
    double E = 0.0;
    E += InertialEnergy::compute_energy(x, m_x_hat, m_state.mass_diag, m_config.dt);
    E += GravityEnergy::compute_energy(x, m_state.mass_diag, m_config.gravity);
    for (const auto& body : m_tet_bodies) {
        E += MaterialEnergy<FixedCorotatedMaterial>::compute_energy(body, x, m_material);
    }
    // Day 3: += BarrierEnergy::compute_energy(...)
    return E;
}
```

gradient 同理，三项用 `+=` 模式累加：

```cpp
void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.setZero();
    InertialEnergy::compute_gradient(x, m_x_hat, m_state.mass_diag, m_config.dt, gradient);
    GravityEnergy::compute_gradient(m_state.mass_diag, m_config.gravity, gradient);
    for (const auto& body : m_tet_bodies) {
        MaterialEnergy<FixedCorotatedMaterial>::compute_gradient(body, x, m_material, gradient);
    }
}
```

Hessian 只有 inertial + elastic 贡献（gravity 是线性能量，Hessian 为零）：

```cpp
void compute_total_hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) {
    triplets.clear();
    InertialEnergy::compute_hessian_triplets(m_state.mass_diag, m_config.dt, triplets);
    // GravityEnergy: 无 Hessian 贡献（线性能量）
    for (const auto& body : m_tet_bodies) {
        MaterialEnergy<FixedCorotatedMaterial>::compute_hessian_triplets(body, x, m_material, triplets);
    }
}
```

### Day 3 扩展点

Day 3 加 contact 时，在总能量装配里加：

```cpp
E += kappa * BarrierEnergy::compute_energy(collision_constraints, x, dhat);
```

梯度和 Hessian 同理。CCD 在 line search 里加 `alpha_max` 限制。

### 与现有系统的关系

Day 1 不修改 `PhysicsSystem`。IPCSystem 独立存在，只在测试和 demo 中直接使用。

Day 5 或更晚时再接入：

```cpp
class PhysicsSystem {
    ClothWorld m_cloth_world;
    RigidBodyWorld m_rigid_body_world;
    IPCSystem m_ipc_system;  // 新增
    // ...
};
```

验收：
- `initialize()` 后 `state()` 有正确大小
- `step()` 一次后顶点位置有变化
- stick DBC vertices 保持完全固定
- slip DBC vertices 在约束轴上不动，自由轴可以移动
- 多步 step 后不出 NaN
