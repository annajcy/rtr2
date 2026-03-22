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

    std::unique_ptr<TetMaterialModel> m_material;

    Eigen::VectorXd m_x_hat;  // 预测位置
    std::vector<bool> m_free_dof_mask;

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
5. 对 fixed vertices: state.v[fixed] = 0
```

### compute_x_hat()

```cpp
m_x_hat = m_state.x_prev;
for (std::size_t i = 0; i < m_state.dof_count(); i += 3) {
    m_x_hat[i+0] += m_config.dt * m_state.v[i+0];
    m_x_hat[i+1] += m_config.dt * m_state.v[i+1] + m_config.dt * m_config.dt * m_config.gravity.y();
    m_x_hat[i+2] += m_config.dt * m_state.v[i+2];
}
// gravity 的 x/z 分量同理，如果 gravity 不只是 y 方向
```

### 总能量装配

```cpp
double compute_total_energy(const Eigen::VectorXd& x) {
    double E = 0.0;
    E += InertialEnergy::compute_energy(x, m_x_hat, m_state.mass_diag, m_config.dt);
    for (const auto& body : m_tet_bodies) {
        E += TetElasticAssembler::compute_energy(body, /* x */, *m_material);
    }
    // Day 3: += BarrierEnergy::compute_energy(...)
    return E;
}
```

gradient 和 Hessian 同理，用 `+=` 模式累加。

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
- fixed vertices 保持固定
- 多步 step 后不出 NaN
