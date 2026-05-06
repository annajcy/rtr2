# `inertial_energy.hpp`

`src/rtr/system/physics/ipc/energy/inertial_energy.hpp` implements the pure inertial term used by the backward-Euler energy formulation.

## Physical Background

Implicit Euler time integration turns a time step into an optimization problem. The first term in the total energy is the inertial energy — it penalizes deviation from the inertial prediction.

Intuition: without any external or internal forces, the body would coast to $\hat{x}$ at constant velocity. The inertial energy measures "how far the actual position $x$ deviates from this prediction", weighted by mass — heavier objects are harder to deflect.

## Theory

### From Backward Euler to Energy Minimization

The backward Euler update rule:

$$
M \frac{x^{n+1} - 2x^n + x^{n-1}}{h^2} = f(x^{n+1})
$$

is equivalent to solving (see Lec 1 §4.3):

$$
x^{n+1} = \arg\min_x \; E(x), \qquad E(x) = \underbrace{\frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})}_{E_{inertial}} + h^2 P(x)
$$

where $\hat{x} = x^n + h v^n$ is the pure inertial prediction (no external forces).

### Inertial Energy Definition

$$
E_{inertial}(x) = \frac{1}{2h^2}(x - \hat{x})^T M (x - \hat{x})
$$

$M$ is the lumped-mass diagonal matrix, $h$ is the time step.

This is a **quadratic form** in $x$, expanded per DOF:

$$
E_{inertial} = \frac{1}{2h^2} \sum_{i=0}^{3n-1} m_i (x_i - \hat{x}_i)^2
$$

where $m_i = \text{mass\_diag}[i]$.

### Gradient

Differentiating the quadratic:

$$
\frac{\partial E_{inertial}}{\partial x_i} = \frac{m_i}{h^2}(x_i - \hat{x}_i)
$$

In vector form:

$$
\boxed{\nabla E_{inertial}(x) = \frac{M}{h^2}(x - \hat{x})}
$$

Physical meaning: the gradient points from $\hat{x}$ toward $x$ — if $x$ deviates from the inertial prediction, the gradient pulls it back. At $x = \hat{x}$, the gradient is zero (the inertial prediction is the minimizer of the inertial energy alone).

### Hessian

Second derivative:

$$
\frac{\partial^2 E_{inertial}}{\partial x_i \partial x_j} = \begin{cases} m_i / h^2, & i = j \\ 0, & i \neq j \end{cases}
$$

$$
\boxed{H_{inertial} = \frac{M}{h^2}}
$$

This is a **diagonal matrix**, always **positive definite** ($m_i > 0$, $h > 0$).

Key property (Lec 2 §3.2): the inertial Hessian never needs PSD projection — it is naturally positive definite. This is the foundation for Newton solver convergence: even if the elastic Hessian has negative eigenvalues, the inertial term ensures the total Hessian is "not too bad".

### Meaning of $\hat{x}$

In this project, $\hat{x}$ is the **pure inertial prediction**:

$$
\hat{x} = x_{prev} + h \cdot v
$$

It does **not** include $h^2 M^{-1} f_{ext}$. Gravity is handled by the separate `GravityEnergy`. Both approaches are mathematically equivalent; the latter is chosen for semantic clarity and extensibility.

## API

```cpp
struct InertialEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& x_hat;
        const Eigen::VectorXd& mass_diag;
        double dt;
    };

    static double compute_energy(const Input& input);
    static void compute_gradient(const Input& input, Eigen::VectorXd& gradient);
    static void compute_hessian_triplets(const Input& input,
                                         std::vector<Eigen::Triplet<double>>& triplets);
};
```

## Implementation

### `compute_energy`

$$
E = \frac{1}{2h^2} \sum_i m_i (x_i - \hat{x}_i)^2
$$

```cpp
static double compute_energy(const Input& input) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    const VectorXd delta = input.x - input.x_hat;
    return 0.5 * inv_dt_sq * input.mass_diag.dot(delta.cwiseProduct(delta));
}
```

`mass_diag.dot(delta.cwiseProduct(delta))` equals $\sum_i m_i \delta_i^2$ — element-wise square then dot with mass gives the weighted norm squared $\|x - \hat{x}\|_M^2$.

### `compute_gradient`

$$
g_i \mathrel{+}= \frac{m_i}{h^2}(x_i - \hat{x}_i)
$$

```cpp
static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    gradient.array() += inv_dt_sq * input.mass_diag.array() * (input.x - input.x_hat).array();
}
```

Uses Eigen's `.array()` for element-wise operations, avoiding explicit loops. `+=` mode — gradient accumulates onto existing values, supporting multi-energy-term summation.

### `compute_hessian_triplets`

$$
H_{ii} = \frac{m_i}{h^2}, \qquad H_{ij} = 0 \; (i \neq j)
$$

```cpp
static void compute_hessian_triplets(const Input& input,
                                     std::vector<Eigen::Triplet<double>>& triplets) {
    const double inv_dt_sq = 1.0 / (input.dt * input.dt);
    triplets.reserve(triplets.size() + static_cast<size_t>(input.mass_diag.size()));
    for (Eigen::Index i = 0; i < input.mass_diag.size(); ++i) {
        triplets.emplace_back(i, i, inv_dt_sq * input.mass_diag[i]);
    }
}
```

Emits only $3n$ diagonal triplets (one per DOF). `reserve` pre-allocates to avoid repeated reallocation during push_back.

## Numerical Verification

- $x = \hat{x}$: $E = 0$, $\nabla E = 0$ ✓
- $x \neq \hat{x}$: gradient points along $(x - \hat{x})$, the direction of deviation ✓
- Hessian is always positive definite (diagonal entries = $m_i / h^2 > 0$) ✓
- FD verification: central-difference gradient of a quadratic is exact (tolerance ~$10^{-10}$)
