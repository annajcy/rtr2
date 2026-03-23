# `ipc_system.hpp`

`src/rtr/system/physics/ipc/core/ipc_system.hpp` is the top-level orchestrator of the IPC deformable body simulation. It connects data structures, energy modules, and the Newton solver into a single `step()` interface.

## Theoretical Foundation: Backward Euler as Optimization

### From Newton's Second Law to an Optimization Problem

The continuous equation of motion is $M\ddot{x} = f(x)$, where $f$ includes internal elastic forces and external forces (gravity). Discretizing with backward Euler at time step $h$:

$$
M \frac{x^{n+1} - 2x^n + x^{n-1}}{h^2} = f(x^{n+1})
$$

Rearranging:

$$
M(x^{n+1} - \hat{x}) = h^2 f(x^{n+1})
$$

where $\hat{x} = 2x^n - x^{n-1} = x^n + h v^n$ is the **inertial prediction** (position if no forces acted).

**Key insight**: if all forces derive from potentials ($f = -\nabla \Psi$), this equation is the stationarity condition of:

$$
\boxed{x^{n+1} = \arg\min_x \underbrace{\frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})}_{E_{\text{inertial}}} + \underbrace{\Psi_{\text{elastic}}(x)}_{E_{\text{elastic}}} + \underbrace{(-f_g^T x)}_{E_{\text{gravity}}} }
$$

**Proof**: set the gradient of the total energy to zero:

$$
\nabla E = \frac{M}{h^2}(x - \hat{x}) + \nabla\Psi - f_g = 0
$$

$$
M(x - \hat{x}) = h^2(-\nabla\Psi + f_g) = h^2 f(x)
$$

which is exactly the backward Euler equation.

### Why Optimization-Based

| Approach | Pros | Cons |
|----------|------|------|
| Direct solve $Ax = b$ | Simple | Only works for linear systems |
| Fixed-point iteration | Easy to implement | Slow convergence for stiff problems |
| **Optimization** | Handles nonlinearity, guarantees energy decrease | Needs energy/gradient/Hessian |

The optimization formulation:
1. **Naturally handles nonlinear elasticity** — no linearization approximation
2. **Energy decrease is guaranteed** by line search — unconditionally stable
3. **Extends cleanly to IPC** — barrier energy is just another term in the sum

## Architecture: How Everything Connects

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
│  ┌──────────────── Total Energy ────────────────────┐   │
│  │                                                    │   │
│  │  InertialEnergy  +  GravityEnergy  +  Σ Material  │   │
│  │  (quadratic)       (linear)          (nonlinear)  │   │
│  │                                                    │   │
│  │  Each provides: energy / gradient / hessian_triplets│   │
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
│               state.x updated                            │
│               state.v = (x - x_prev) / dt               │
└─────────────────────────────────────────────────────────┘
```

### Layer Responsibilities

| Layer | Files | Responsibility |
|-------|-------|---------------|
| **Data** | `ipc_state.hpp`, `tet_body.hpp` | Global DOF vector, per-body geometry/material/constraints |
| **Energy** | `inertial_energy.hpp`, `gravity_energy.hpp`, `material_energy.hpp` | Per-term energy/gradient/Hessian computation |
| **Material** | `tet_fixed_corotated.hpp`, `tet_material_variant.hpp` | Constitutive model ($F \to \Psi, P, \partial^2\Psi$) |
| **Solver** | `newton_solver.hpp`, `line_search.hpp` | Nonlinear solve with DBC and line search |
| **System** | `ipc_system.hpp` | Orchestration: assemble, solve, update |

## `IPCConfig`

```cpp
struct IPCConfig {
    double dt{0.01};                               // fixed time step (seconds)
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};     // gravity acceleration
    NewtonSolverParams solver_params{};             // Newton iteration control
};
```

| Parameter | Default | Notes |
|-----------|---------|-------|
| `dt` | 0.01 | IPC uses fixed-size steps. Smaller = more accurate but more steps. Typical: 0.005–0.02. |
| `gravity` | $(0, -9.81, 0)$ | Standard Earth gravity. Set to zero for zero-g tests. |
| `solver_params` | See Newton doc | `max_iterations=50`, `gradient_tolerance=1e-6`, etc. |

## Runtime Assembly and `initialize()`

### What It Does

```cpp
void rebuild_runtime_state() {
    // Phase 1: Preserve old per-body state by IPCBodyID
    // Phase 2: Per-body precompute and DOF reassignment
    for (auto body_id : m_body_order) {
        auto& body = m_tet_bodies.at(body_id);
        body.precompute();           // Dm_inv, rest_volumes, vertex_masses
        body.info.dof_offset = dof_offset;
        body.info.vertex_count = body.vertex_count();
        total_vertices += body.vertex_count();
        dof_offset += 3u * body.vertex_count();
    }

    // Phase 3: Global state assembly
    m_state.resize(total_vertices);  // allocate x, x_prev, v, mass_diag (all 3n)

    // Phase 4: Restore old states when IPCBodyID still exists
    //          otherwise initialize from rest positions
    for (auto body_id : m_body_order) {
        const auto& body = m_tet_bodies.at(body_id);
    }

    // Phase 5: Build DBC mask
    compute_x_hat();
    build_free_dof_mask();
    m_initialized = true;
}
```

`initialize()` is now just the explicit entry point for this rebuild. Registration changes (`create_tet_body`, `remove_tet_body`) mark the system dirty, and `step()` automatically rebuilds before solving.

`body.precompute()` reads density from each body's material object, so `mass_diag` is assembled from `TetBody::material`.

### DOF Offset Layout

With $B$ bodies having $n_0, n_1, \ldots, n_{B-1}$ vertices:

```
Global DOF vector x (size 3N, where N = Σn_b):

┌─── body 0 ────┐┌─── body 1 ────┐┌─── body 2 ────┐
[x0 y0 z0 x1 y1 z1 ...][x0 y0 z0 ...][x0 y0 z0 ...]
 ↑                       ↑              ↑
 dof_offset=0            dof_offset     dof_offset
                         =3*n_0         =3*(n_0+n_1)
```

`body.info.dof_offset` tells where body's DOFs start in the global vector. All energy modules use this to index into $x$.

### `build_free_dof_mask()`

```cpp
void build_free_dof_mask() {
    m_free_dof_mask.assign(m_state.dof_count(), true);  // all free by default
    for (const auto& body : m_tet_bodies) {
        for each vertex in body.fixed_vertices:
            if (fixed):
                m_free_dof_mask[3*global_vertex + 0] = false;  // x
                m_free_dof_mask[3*global_vertex + 1] = false;  // y
                m_free_dof_mask[3*global_vertex + 2] = false;  // z
    }
}
```

This expands per-vertex constraints to per-DOF booleans. The Newton solver uses this mask to eliminate constrained DOFs from the linear system (see `newton_solver.hpp` doc).

## Time Stepping: `step()`

### The Complete Flow

```cpp
void step() {
    // 1. Save previous state
    m_state.x_prev = m_state.x;

    // 2. Inertial prediction
    compute_x_hat();   // x_hat = x_prev + dt * v

    // 3. Assemble Newton callbacks
    NewtonProblem problem{
        .compute_energy = [this](x) { return compute_total_energy(x); },
        .compute_gradient = [this](x, g) { compute_total_gradient(x, g); },
        .compute_hessian_triplets = [this](x, t) { compute_total_hessian(x, t); },
    };

    // 4. Newton solve (modifies m_state.x in place)
    solve(m_state, m_x_hat, m_free_dof_mask, problem, m_config.solver_params);

    // 5. Update velocity
    m_state.v = (m_state.x - m_state.x_prev) / m_config.dt;

    // 6. Zero constrained velocities
    for each DOF i:
        if (!m_free_dof_mask[i]):
            m_state.v[i] = 0.0;
}
```

### Phase-by-Phase Theory

#### Phase 1: Save Previous State

$x^{n-1} \leftarrow x^n$

Needed for velocity computation and the inertial energy term.

#### Phase 2: Inertial Prediction

$$
\hat{x} = x^n + h \cdot v^n
$$

```cpp
void compute_x_hat() {
    m_x_hat = m_state.x_prev + m_config.dt * m_state.v;
}
```

$\hat{x}$ is the position the system would reach if no forces acted. It's the "momentum carry-forward" from the previous step. Gravity and elastic forces will pull $x^{n+1}$ away from $\hat{x}$ during the Newton solve.

**Why not include gravity in $\hat{x}$**: See `gravity_energy.hpp` doc. Keeping $\hat{x}$ as pure kinematics makes the energy decomposition cleaner and each term independently debuggable.

#### Phase 3: Newton Problem Assembly

The `NewtonProblem` is a callback bundle that the solver uses to evaluate the total energy landscape:

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

The lambdas capture `this` so the solver can call back into `IPCSystem`'s energy assembly without knowing about specific energy terms. This decoupling means the solver code doesn't change when new energy terms are added.

#### Phase 4: Newton Solve

```cpp
const NewtonSolverResult result =
    solve(m_state, m_x_hat, m_free_dof_mask, problem, m_config.solver_params);
```

The solver modifies `m_state.x` in place through iterative Newton steps (see `newton_solver.hpp` doc). On return, `m_state.x` contains $x^{n+1}$.

If the solve doesn't converge, a warning is logged but the simulation continues with whatever $x$ was reached — this is common for difficult configurations and usually self-corrects in subsequent steps.

#### Phase 5-6: Velocity Update

$$
v^{n+1} = \frac{x^{n+1} - x^n}{h}
$$

```cpp
m_state.v = (m_state.x - m_state.x_prev) / m_config.dt;
```

Then zero the velocity on constrained DOFs:

```cpp
for (Eigen::Index i = 0; i < m_state.v.size(); ++i) {
    if (!m_free_dof_mask[static_cast<std::size_t>(i)]) {
        m_state.v[i] = 0.0;
    }
}
```

**Why zero constrained velocities**: without this, fixed vertices would accumulate fictitious velocity from floating-point noise in $x^{n+1} - x^n$, and $\hat{x}$ would drift in subsequent steps.

## Total Energy Assembly

### Energy

$$
E(x) = E_{\text{inertial}}(x) + E_{\text{gravity}}(x) + \sum_{b} E_{\text{material}}^{(b)}(x)
$$

```cpp
double compute_total_energy(const Eigen::VectorXd& x) const {
    double total_energy = 0.0;

    // Inertial: (1/2h²) (x - x_hat)^T M (x - x_hat)
    total_energy += InertialEnergy::compute_energy(InertialEnergy::Input{
        .x = x, .x_hat = m_x_hat, .mass_diag = m_state.mass_diag, .dt = m_config.dt,
    });

    // Gravity: -f_g^T x = -Σ m_i * g * x_i
    total_energy += GravityEnergy::compute_energy(GravityEnergy::Input{
        .x = x, .mass_diag = m_state.mass_diag, .gravity = m_config.gravity,
    });

    // Elastic: Σ_body Σ_tet V_e * Ψ(F)
    for (const auto& body : m_tet_bodies) {
        total_energy += material_energy_variant::compute_energy(body, x);
    }

    return total_energy;
}
```

Each energy term uses the `Energy` concept's `Input` struct pattern — the assembly code just constructs the input and calls the static method.

### Gradient

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

All gradient methods use `+=` accumulation mode — `gradient` is zeroed once, then each term adds its contribution. This avoids allocating temporary vectors.

### Hessian

$$
H = \frac{M}{h^2} + \sum_b H_{\text{material}}^{(b)}
$$

```cpp
void compute_total_hessian(const Eigen::VectorXd& x,
                           std::vector<Eigen::Triplet<double>>& triplets) const {
    triplets.clear();
    InertialEnergy::compute_hessian_triplets({...}, triplets);   // diagonal M/h²
    // GravityEnergy: zero Hessian (linear energy), no contribution
    for (const auto& body : m_tet_bodies) {
        material_energy_variant::compute_hessian_triplets(body, x, triplets);  // per-tet 12×12
    }
}
```

Note: gravity contributes no Hessian (linear energy → constant gradient → zero Hessian). This is why `GravityEnergy::compute_hessian_triplets` is a no-op.

### Variant Dispatch

Each body carries its own `TetMaterialVariant material` field. The `material_energy_variant` functions use `std::visit` to dispatch:

```cpp
inline double compute_energy(const TetBody& body, const Eigen::VectorXd& x) {
    return std::visit([&](const auto& material) {
        return MaterialEnergy<std::decay_t<decltype(material)>>::compute_energy(
            {.body = body, .x = x, .material = material});
    }, body.material);
}
```

The lambda's `const auto&` parameter means the compiler generates one specialization per variant alternative. The inner tet loop runs as fully-inlined template code — no virtual dispatch per tet.

## Hessian Sparsity Structure

The assembled Hessian has a specific sparsity pattern:

```
  ┌─────────────────────────────┐
  │ M/h² (diagonal)             │  ← InertialEnergy
  │  +                          │
  │ ┌───┐                       │
  │ │12×12│     (tet 0)         │  ← MaterialEnergy
  │ └───┘                       │
  │       ┌───┐                 │
  │       │12×12│ (tet 1)       │
  │       └───┘                 │
  │             ⋱               │
  │                  ┌───┐      │
  │                  │12×12│    │
  │                  └───┘      │
  └─────────────────────────────┘
```

- **Inertial**: $3N$ diagonal entries $m_i / h^2$
- **Elastic**: per-tet $12 \times 12$ blocks (4 vertices × 3 DOFs each), overlapping at shared vertices

The triplets from different terms are accumulated by `setFromTriplets` with a sum functor in the Newton solver.

## State Access

```cpp
const IPCState& state() const;                              // full global state
bool has_tet_body(IPCBodyID id) const;
const TetBody& get_tet_body(IPCBodyID id) const;            // body by stable ID
std::vector<Eigen::Vector3d> get_body_positions(IPCBodyID body_id) const;
```

`get_body_positions` extracts a body's current vertex positions from the global DOF vector:

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

This is used by the scene sync layer to write back deformed positions to `DeformableMeshComponent` for rendering.

## Complete Data Flow: One Time Step

```
                    IPCSystem::step()
                          │
          ┌───────────────┼───────────────┐
          ▼               ▼               ▼
     x_prev = x     compute_x_hat()   build callbacks
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
                    │   H_FF · Δx_F = -g_F  (reduced)    │
                    │   Δx ← expand(Δx_F)                │
                    │   α ← line_search(E, x, Δx)        │
                    │   x ← x + α·Δx                     │
                    └─────────────────────────────────────┘
                          │
                          ▼
                    v = (x - x_prev) / h
                    zero constrained v
                          │
                          ▼
                    state.x, state.v ready
                          │
                          ▼
                    (scene sync reads state.x
                     → tet_to_mesh → DeformableMeshComponent
                     → GPU upload → render)
```

## Numerical Example: Single Tet Free Fall

Setup: 1 tet (4 vertices), $h = 0.01$, $g = (0, -9.81, 0)$, $E = 10^5$, $\nu = 0.3$, no DBC.

**Step 0 (initialize)**:
- `x = x_prev = rest_positions` (all at rest)
- `v = 0`
- `mass_diag`: each vertex gets $\rho V_e / 4$ repeated 3 times

**Step 1**:
- `x_hat = x + 0 = x` (no velocity yet)
- Newton iteration 1:
  - $E_{\text{inertial}} = 0$ (x = x_hat)
  - $E_{\text{gravity}} = -\sum m_i g_y y_i < 0$ (below reference)
  - $\nabla E_{\text{gravity}} = (0, +m \cdot 9.81, 0)$ per vertex (upward)
  - $\nabla E_{\text{inertial}} = 0$
  - Newton step pushes vertices downward (opposite to gradient)
  - After line search: vertices move down by $\approx \frac{1}{2} g h^2 \approx 4.9 \times 10^{-4}$ m
- Velocity update: $v_y \approx -9.81 \times 0.01 = -0.0981$ m/s

**Step 2**:
- `x_hat = x + 0.01 * v` (includes momentum from step 1)
- Gravity continues to accelerate: $v_y \approx -0.1962$ m/s
- Tet slightly stretched by inertia vs elastic restoring force

## Error Handling

| Situation | Response |
|-----------|----------|
| `step()` before any explicit `initialize()` | Auto-rebuilds if bodies exist |
| Empty body list | `step()` returns immediately (no-op) |
| Newton doesn't converge | Warning log, simulation continues |
| `get_body_positions` before init | `throw std::logic_error` |
| Missing `IPCBodyID` | `throw std::out_of_range` (from `.at()`) |

## Extension Points

| Extension | Where to modify |
|-----------|----------------|
| New energy term (barrier, friction) | Add to `compute_total_energy/gradient/hessian` |
| New material model | Add to `TetMaterialVariant`, implement `TetMaterialModel` concept |
| Per-axis DBC (slip) | Change `build_free_dof_mask` to read `AxisConstraint` instead of `fixed_vertices` |
| CCD step clamping | Pass `alpha_max` to line search in Newton callback |
| Multi-body contact | Add contact pair detection before Newton, barrier energy term |
