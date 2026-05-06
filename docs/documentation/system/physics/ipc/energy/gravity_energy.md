# `gravity_energy.hpp`

`src/rtr/system/physics/ipc/energy/gravity_energy.hpp` implements gravity as a separate linear potential energy term.

## Physical Background

In the optimization framework, external forces enter the objective through their potential energy. A constant force $f$ corresponds to potential $-f^T x$ (a linear function) — the farther you move along the force direction, the lower the potential.

Gravity is a prototypical constant force: each node experiences $f_a = m_a \mathbf{g}$.

## Theory

### Why Not Fold Gravity into $\hat{x}$

Two equivalent approaches:

**Approach A**: fold gravity into the inertial prediction: $\hat{x} = x_{prev} + hv + h^2 M^{-1} f_g$

**Approach B**: keep $\hat{x}$ as pure inertial, add gravity as a separate energy term

Equivalence proof: Approach B's gradient = $\frac{M}{h^2}(x - \hat{x}) - f_g + \nabla\Psi$. Setting gradient to zero → $\frac{M}{h^2}(x - (\hat{x} + h^2 M^{-1}f_g)) + \nabla\Psi = 0$, which is exactly Approach A.

Approach B is chosen because:
- $\hat{x}$ has a clean semantic meaning (pure kinematics)
- Adding new external forces = appending energy terms, no change to $\hat{x}$ computation
- Each energy term can be logged independently for debugging

### Energy

Gravity force on node $a$: $f_a = m_a \mathbf{g}$ (3D vector). The corresponding potential:

$$
E_{gravity}(x) = -\sum_a m_a \mathbf{g}^T \mathbf{x}_a = -f_g^T x
$$

Expanded in the flattened $3n$ layout (with $\mathbf{g} = (g_x, g_y, g_z)$):

$$
E_{gravity} = -\sum_{i=0}^{n-1} \big( m_{3i} g_x x_{3i} + m_{3i+1} g_y x_{3i+1} + m_{3i+2} g_z x_{3i+2} \big)
$$

Since lumped mass assigns the same value to all 3 DOFs of each vertex ($m_{3i} = m_{3i+1} = m_{3i+2}$):

$$
E_{gravity} = -\sum_{i=0}^{n-1} m_{3i} (g_x x_{3i} + g_y x_{3i+1} + g_z x_{3i+2})
$$

This is a **linear function** of $x$.

### Gradient

The derivative of a linear function is constant:

$$
\frac{\partial E_{gravity}}{\partial x_k} = -m_k \cdot g_{k \bmod 3}
$$

In vector form (per vertex $a$):

$$
\boxed{\nabla_{x_a} E_{gravity} = -m_a \mathbf{g}}
$$

Physical meaning: the energy gradient is the negative of the gravitational force. The gradient points upward (opposite to gravity). This makes sense — gravity pulls objects downward (decreasing potential), so the gradient points upward (increasing potential).

**Key property**: the gradient is constant (independent of $x$). It can be precomputed once at `initialize()` and accumulated each step.

### Hessian

The derivative of a constant is zero:

$$
\boxed{H_{gravity} = 0}
$$

Gravity contributes no Hessian triplets. This is why `compute_hessian_triplets` is a no-op.

Physical meaning: gravity does not change the system's stiffness — the force is the same regardless of position.

## API

```cpp
struct GravityEnergy {
    struct Input {
        const Eigen::VectorXd& x;
        const Eigen::VectorXd& mass_diag;
        const Eigen::Vector3d& gravity;
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
E = -\sum_i m_i \cdot g_{i \bmod 3} \cdot x_i
$$

```cpp
static double compute_energy(const Input& input) {
    double energy = 0.0;
    for (Eigen::Index i = 0; i < input.x.size(); i += 3) {
        energy -= input.mass_diag[i + 0] * input.gravity.x() * input.x[i + 0];
        energy -= input.mass_diag[i + 1] * input.gravity.y() * input.x[i + 1];
        energy -= input.mass_diag[i + 2] * input.gravity.z() * input.x[i + 2];
    }
    return energy;
}
```

Strides by vertex (`i += 3`), processing x/y/z components each step. In the typical scenario, `gravity.x()` = 0, `gravity.y()` = $-9.81$, `gravity.z()` = 0.

### `compute_gradient`

$$
g_k \mathrel{-}= m_k \cdot g_{k \bmod 3}
$$

```cpp
static void compute_gradient(const Input& input, Eigen::VectorXd& gradient) {
    for (Eigen::Index i = 0; i < gradient.size(); i += 3) {
        gradient[i + 0] -= input.mass_diag[i + 0] * input.gravity.x();
        gradient[i + 1] -= input.mass_diag[i + 1] * input.gravity.y();
        gradient[i + 2] -= input.mass_diag[i + 2] * input.gravity.z();
    }
}
```

`-=` because $\nabla E = -m\mathbf{g}$ (for the y component: $-m \cdot (-9.81) = +9.81m$, i.e., the gradient points upward).

`+=` mode: gradient accumulates onto existing values. For $\mathbf{g} = (0, -9.81, 0)$:

- x component: $g_{3i+0} -= m \cdot 0 = 0$ (unchanged)
- y component: $g_{3i+1} -= m \cdot (-9.81) = g_{3i+1} + 9.81m$ (upward gradient)
- z component: $g_{3i+2} -= m \cdot 0 = 0$ (unchanged)

### `compute_hessian_triplets`

```cpp
static void compute_hessian_triplets(const Input& input,
                                     std::vector<Eigen::Triplet<double>>& triplets) {
    validate_inputs(input);
    (void)triplets;  // deliberate no-op
}
```

Hessian is zero. This function exists to satisfy the `Energy` concept's uniform interface, so generic assembly code does not need a special branch for gravity.

## Numerical Verification

- $\mathbf{g} = (0,0,0)$: $E = 0$, $\nabla E = 0$ ✓
- $\mathbf{g} = (0, -9.81, 0)$:
  - Only y components contribute to the gradient
  - $\nabla E$ y-component = $+m \cdot 9.81$ (upward, because moving up increases potential) ✓
  - Energy = $+9.81 \sum_a m_a y_a$ (higher $y$ = higher energy) ✓
- FD verification: gradient of a linear function is exact to machine precision ($\sim 10^{-14}$)
- Hessian contributes no triplets ✓
