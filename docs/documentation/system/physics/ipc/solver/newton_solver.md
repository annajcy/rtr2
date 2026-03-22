# `newton_solver.hpp`

`src/rtr/system/physics/ipc/solver/newton_solver.hpp` implements the Newton solver for the IPC optimization problem.

## Background: Why Newton's Method

The IPC time step is formulated as an unconstrained optimization:

$$
x^{k+1} = \arg\min_x E(x)
$$

where $E(x) = E_{\text{inertial}}(x) + E_{\text{gravity}}(x) + E_{\text{elastic}}(x) + \ldots$

Newton's method finds the minimum by repeatedly solving a local quadratic approximation.

### Quadratic Approximation

At iteration $k$, approximate $E$ around the current $x^k$ by its second-order Taylor expansion:

$$
E(x^k + \Delta x) \approx E(x^k) + g^T \Delta x + \frac{1}{2} \Delta x^T H \Delta x
$$

where $g = \nabla E(x^k)$ is the gradient and $H = \nabla^2 E(x^k)$ is the Hessian.

Setting $\nabla_{\Delta x} = 0$:

$$
g + H \Delta x = 0
$$

$$
\boxed{H \Delta x = -g}
$$

This is the **Newton equation**. Solving it gives the step $\Delta x$ that minimizes the local quadratic model.

### Why Newton Converges Quickly

**Theorem (Quadratic Convergence)**: If $H$ is Lipschitz continuous and positive definite near the minimizer $x^*$, and $x^0$ is close enough, then:

$$
\|x^{k+1} - x^*\| \le C \|x^k - x^*\|^2
$$

This means the number of correct digits roughly doubles each iteration. For IPC, convergence typically takes 5-20 iterations per time step.

### Newton vs Gradient Descent

| Property | Newton | Gradient Descent |
|----------|--------|-----------------|
| Step | $\Delta x = -H^{-1} g$ | $\Delta x = -g$ |
| Convergence | Quadratic | Linear |
| Cost per iteration | $O(n)$ sparse solve | $O(n)$ vector ops |
| Stiffness sensitivity | Low (Hessian adapts) | High (needs small step) |

For stiff elastic problems (large Young's modulus), gradient descent needs tiny steps. Newton handles stiffness naturally because the Hessian encodes the local curvature.

## Data Structures

### `NewtonSolverParams`

```cpp
struct NewtonSolverParams {
    int max_iterations{50};        // max Newton iterations per time step
    double gradient_tolerance{1e-6}; // ||g||_inf convergence threshold
    double dx_tolerance{1e-8};      // ||Δx||_inf convergence threshold
    double regularization{1e-8};    // diagonal regularization ε
    bool use_psd_projection{false}; // reserved for future PSD projection
};
```

### `NewtonSolverResult`

```cpp
struct NewtonSolverResult {
    int iterations{0};              // actual iterations taken
    double final_gradient_norm{0.0}; // ||g||_inf at termination
    double final_energy{0.0};       // E(x) at termination
    bool converged{false};          // true if a convergence criterion was met
};
```

### `NewtonProblem`

```cpp
struct NewtonProblem {
    std::function<double(const Eigen::VectorXd& x)> compute_energy;
    std::function<void(const Eigen::VectorXd& x, Eigen::VectorXd& gradient)> compute_gradient;
    std::function<void(const Eigen::VectorXd& x,
                       std::vector<Eigen::Triplet<double>>& triplets)> compute_hessian_triplets;
};
```

The solver is decoupled from specific energy models through callbacks. `IPCSystem` binds these to its `compute_total_energy/gradient/hessian` methods.

## Dirichlet Boundary Conditions: Reduced Solve

### The Mathematical Formulation

Given a free DOF set $\mathcal{F}$ and constrained DOF set $\mathcal{C}$ (with $\mathcal{F} \cup \mathcal{C} = \{0, \ldots, 3n-1\}$), the constrained Newton step requires $\Delta x_i = 0$ for $i \in \mathcal{C}$.

Partitioning the system:

$$
\begin{bmatrix} H_{FF} & H_{FC} \\ H_{CF} & H_{CC} \end{bmatrix}
\begin{bmatrix} \Delta x_F \\ 0 \end{bmatrix}
= -\begin{bmatrix} g_F \\ g_C \end{bmatrix}
$$

The first block row gives:

$$
\boxed{H_{FF} \Delta x_F = -g_F}
$$

This is the **reduced system** — only the free-free sub-block of the Hessian, and only the free components of the gradient.

### Implementation: Index Remapping

**Step 1**: Collect free DOF indices.

```cpp
inline std::vector<Eigen::Index> collect_free_dofs(const std::vector<bool>& free_dof_mask) {
    std::vector<Eigen::Index> free_dofs{};
    free_dofs.reserve(free_dof_mask.size());
    for (std::size_t i = 0; i < free_dof_mask.size(); ++i) {
        if (free_dof_mask[i]) {
            free_dofs.push_back(static_cast<Eigen::Index>(i));
        }
    }
    return free_dofs;
}
```

If the global DOF count is $3n$ and $m$ DOFs are free, `free_dofs` has $m$ entries.

**Step 2**: Zero out the gradient on fixed DOFs.

```cpp
inline void zero_fixed_gradient(Eigen::VectorXd& gradient, const std::vector<bool>& free_dof_mask) {
    for (Eigen::Index i = 0; i < gradient.size(); ++i) {
        if (!free_dof_mask[static_cast<std::size_t>(i)]) {
            gradient[i] = 0.0;
        }
    }
}
```

This ensures $g_C = 0$ in the full gradient, so the masked infinity norm only reflects free DOFs.

**Step 3**: Build the reduced Hessian.

```cpp
inline Eigen::SparseMatrix<double> build_reduced_hessian(
    Eigen::Index dof_count,
    const std::vector<Eigen::Triplet<double>>& full_triplets,
    const std::vector<Eigen::Index>& free_dofs,
    double regularization)
```

The process:

1. Build a mapping `reduced_index[global_i]` → reduced row/col index (or $-1$ for fixed DOFs).

```cpp
std::vector<Eigen::Index> reduced_index(dof_count, -1);
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_index[free_dofs[i]] = i;
}
```

2. Filter triplets: keep only `(i, j, v)` where both `i` and `j` are free, remapping to reduced indices.

```cpp
for (const auto& triplet : full_triplets) {
    const Eigen::Index reduced_row = reduced_index[triplet.row()];
    const Eigen::Index reduced_col = reduced_index[triplet.col()];
    if (reduced_row < 0 || reduced_col < 0) continue;  // involves a fixed DOF
    reduced_triplets.emplace_back(reduced_row, reduced_col, triplet.value());
}
```

3. Add diagonal regularization $\varepsilon I$:

```cpp
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_triplets.emplace_back(i, i, regularization);
}
```

4. Assemble sparse matrix with duplicate summation:

```cpp
reduced_hessian.setFromTriplets(reduced_triplets.begin(), reduced_triplets.end(),
    [](const double lhs, const double rhs) { return lhs + rhs; });
```

The custom accumulator ensures duplicate entries (from overlapping element stiffness matrices) are summed, not overwritten.

**Step 4**: Extract the reduced gradient.

```cpp
Eigen::VectorXd reduced_gradient(free_dofs.size());
for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
    reduced_gradient[i] = gradient[free_dofs[i]];
}
```

**Step 5**: Solve the reduced system.

```cpp
inline bool solve_reduced_system(const Eigen::SparseMatrix<double>& reduced_hessian,
                                 const Eigen::VectorXd& reduced_gradient,
                                 Eigen::VectorXd& reduced_dx) {
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver{};
    solver.compute(reduced_hessian);
    if (solver.info() != Eigen::Success) return false;

    reduced_dx = solver.solve(-reduced_gradient);
    if (solver.info() != Eigen::Success || !reduced_dx.allFinite()) return false;
    return true;
}
```

`SimplicialLDLT` performs $LDL^T$ factorization (Cholesky-like for symmetric matrices, not requiring positive definiteness — the $D$ can have negative entries). Complexity: $O(m)$ for sparse matrices with bounded bandwidth.

**Step 6**: Expand back to full DOF space.

```cpp
inline Eigen::VectorXd expand_reduced_step(Eigen::Index dof_count,
                                           const std::vector<Eigen::Index>& free_dofs,
                                           const Eigen::VectorXd& reduced_dx) {
    Eigen::VectorXd dx = Eigen::VectorXd::Zero(dof_count);
    for (Eigen::Index i = 0; i < free_dofs.size(); ++i) {
        dx[free_dofs[i]] = reduced_dx[i];
    }
    return dx;
}
```

Fixed DOFs get $\Delta x_i = 0$ (from the `Zero` initialization).

## Convergence Criteria

The solver uses three convergence criteria, checked at different points:

### 1. Gradient Norm Convergence

$$
\|g\|_\infty = \max_{i \in \mathcal{F}} |g_i| \le \varepsilon_g
$$

```cpp
const double gradient_norm = masked_inf_norm(gradient, free_dof_mask);
if (gradient_norm <= params.gradient_tolerance) {
    result.converged = true;
    return result;
}
```

**Physical meaning**: at a minimum, $\nabla E = 0$. The infinity norm $\|g\|_\infty$ measures the largest force residual on any single DOF. When this is below threshold, every vertex is in approximate equilibrium.

**Why infinity norm**: unlike $\|g\|_2$ which grows with $\sqrt{n}$ (more vertices = larger norm), $\|g\|_\infty$ is independent of mesh resolution.

### 2. Step Size Convergence

$$
\|\Delta x\|_\infty = \max_{i \in \mathcal{F}} |\Delta x_i| \le \varepsilon_{dx}
$$

```cpp
const double dx_norm = masked_inf_norm(dx, free_dof_mask);
if (dx_norm <= params.dx_tolerance) {
    result.converged = true;
    return result;
}
```

**Physical meaning**: the Newton step is negligibly small — positions barely change.

### 3. Post-Update Check

After updating $x$, re-evaluate the gradient:

```cpp
const double post_gradient_norm = evaluate_gradient_norm(problem, state.x, free_dof_mask);
if (post_gradient_norm <= params.gradient_tolerance ||
    line_search_result.alpha * dx_norm <= params.dx_tolerance) {
    result.converged = true;
    return result;
}
```

This catches cases where $\alpha < 1$ makes the actual displacement ($\alpha \cdot \Delta x$) small enough to declare convergence.

## Regularization Strategy

### Why Regularization Is Needed

The Hessian can be indefinite when:
- The energy function is non-convex (e.g., fixed corotated under compression)
- Numerical errors accumulate
- PSD projection is not yet implemented

An indefinite Hessian means the Newton direction may not be a descent direction, and `SimplicialLDLT` may fail.

### Tikhonov Regularization

The solver adds $\varepsilon I$ to the reduced Hessian:

$$
(H_{FF} + \varepsilon I) \Delta x_F = -g_F
$$

This shifts all eigenvalues by $+\varepsilon$, making the matrix "more positive definite."

When $\varepsilon$ is small ($10^{-8}$), this barely affects the Newton direction. When $\varepsilon$ is large, the step approaches $\Delta x \approx -\varepsilon^{-1} g$ (gradient descent).

### Adaptive Retry

```cpp
double regularization = std::max(params.regularization, 0.0);
for (int retry = 0; retry < 6; ++retry) {
    const auto reduced_hessian = build_reduced_hessian(..., regularization);
    solved = solve_reduced_system(reduced_hessian, reduced_gradient, reduced_dx);
    if (solved) break;
    regularization = (regularization > 0.0) ? regularization * 10.0 : 1e-8;
}
```

The retry sequence: $10^{-8} \to 10^{-7} \to 10^{-6} \to 10^{-5} \to 10^{-4} \to 10^{-3}$.

At each retry, the matrix is rebuilt (not just patched) because `setFromTriplets` accumulates — adding more diagonal would double-count.

## Main Solve Loop

### Entry Point

```cpp
inline NewtonSolverResult solve(
    IPCState& state,
    const Eigen::VectorXd& x_hat,
    const std::vector<bool>& free_dof_mask,
    const NewtonProblem& problem,
    const NewtonSolverParams& params = {})
```

### Algorithm in Pseudocode

```
validate inputs
E₀ ← compute_energy(x)
free_dofs ← collect_free_dofs(mask)
if free_dofs is empty: return converged

for k = 0 to max_iterations-1:
    g ← compute_gradient(x)
    zero_fixed_gradient(g, mask)
    if ||g||_inf ≤ ε_g: return converged

    E_k ← compute_energy(x)
    triplets ← compute_hessian_triplets(x)
    g_F ← extract reduced gradient

    // Solve with adaptive regularization
    for retry = 0 to 5:
        H_FF ← build_reduced_hessian(triplets, free_dofs, ε)
        solve H_FF · Δx_F = -g_F
        if success: break
        ε ← ε × 10
    if not solved: return failed

    Δx ← expand_reduced_step(Δx_F)
    if ||Δx||_inf ≤ ε_dx: return converged

    // Line search
    d ← g^T · Δx  (directional derivative)
    α ← backtracking_line_search(α → E(x + α·Δx), E_k, d)
    if line search failed: return failed

    x ← x + α · Δx

    // Post-update convergence check
    if ||g(x)||_inf ≤ ε_g or α·||Δx||_inf ≤ ε_dx: return converged

return result (not converged, max iterations reached)
```

### Line Search Integration

```cpp
const double directional_derivative = gradient.dot(dx);
const auto line_search_result = backtracking_line_search(
    [&state, &problem, &dx](double alpha) {
        const Eigen::VectorXd trial_x = state.x + alpha * dx;
        return problem.compute_energy(trial_x);
    },
    current_energy,
    directional_derivative
);
```

The lambda captures:
- `state.x` — current position
- `dx` — full Newton step (with zeros on fixed DOFs)
- `problem.compute_energy` — the total energy evaluator

For each trial $\alpha$, it constructs $x_{\text{trial}} = x + \alpha \cdot \Delta x$ and evaluates $E(x_{\text{trial}})$.

### Update

```cpp
state.x += line_search_result.alpha * dx;
```

`state.x` is modified in place. After the Newton loop completes, `IPCSystem::step()` uses the updated `state.x` to compute the new velocity.

### Logging

Each iteration logs one line:

```
Newton iter=3 energy=1.234567e+01 |g|_inf=2.345678e-03 |dx|_inf=1.234567e-04 alpha=5.000000e-01 success=1
```

## Computational Complexity

| Operation | Cost | Notes |
|-----------|------|-------|
| `compute_gradient` | $O(n + T)$ | $n$ vertices, $T$ tets |
| `compute_hessian_triplets` | $O(T)$ | Per-tet 12x12 blocks |
| `build_reduced_hessian` | $O(\text{nnz})$ | Filter + remap |
| `SimplicialLDLT::compute` | $O(m^{1.5})$ typical | Sparse fill-in dependent |
| `SimplicialLDLT::solve` | $O(\text{nnz}_L)$ | Back-substitution |
| `backtracking_line_search` | $O(k \cdot (n + T))$ | $k$ energy evaluations |

Total per Newton iteration: dominated by the sparse factorization.
Total per time step: $\sim 5\text{-}20$ Newton iterations $\times$ factorization cost.

## Error Handling

| Condition | Response |
|-----------|----------|
| Non-finite initial energy | `throw std::runtime_error` |
| Non-finite gradient | `throw std::runtime_error` |
| Non-finite Newton step | `throw std::runtime_error` |
| Non-finite updated `state.x` | `throw std::runtime_error` |
| Hessian factorization failure (all retries) | `return result` with `converged = false` |
| Line search failure | `return result` with `converged = false` |
| No free DOFs | `return result` with `converged = true` |
| Hessian triplets out of range | `throw std::out_of_range` |

Non-finite values indicate bugs upstream (energy, gradient, or Hessian computation) and are treated as hard errors. Factorization and line search failures are soft — they return without convergence, and the caller can decide how to proceed.
