# `line_search.hpp`

`src/rtr/system/physics/ipc/solver/line_search.hpp` implements the backtracking Armijo line search used by the IPC Newton solver.

## Why Line Search

Newton's method gives a search direction $\Delta x$ by solving $H \Delta x = -g$. But a full step $x + \Delta x$ can overshoot, especially when:

- The Hessian is only locally accurate (the quadratic model breaks down far from $x^k$)
- The energy landscape is highly nonlinear (large deformations)
- The Hessian has been regularized or projected (the direction is approximate)

Line search scales the step: $x^{k+1} = x^k + \alpha \Delta x$, choosing $\alpha \in (0, 1]$ to ensure **sufficient energy decrease**.

## Theory: Armijo Condition

### The Sufficient Decrease Criterion

A naive check "$E(x + \alpha \Delta x) < E(x)$" is too weak — it accepts tiny decreases that stall convergence. The **Armijo condition** (also called the sufficient decrease condition or first Wolfe condition) requires:

$$
\boxed{E(x + \alpha \Delta x) \le E(x) + c \cdot \alpha \cdot g^T \Delta x}
$$

where:

| Symbol | Meaning | Value |
|--------|---------|-------|
| $E(x)$ | Current energy | — |
| $\alpha$ | Step size | $\in (0, 1]$ |
| $\Delta x$ | Newton direction | $H^{-1}(-g)$ |
| $g$ | Gradient $\nabla E(x)$ | — |
| $c$ | Armijo constant | $10^{-4}$ |
| $g^T \Delta x$ | Directional derivative | Must be $< 0$ |

### Geometric Interpretation

At $\alpha = 0$, the linear model of energy along the search direction is:

$$
\ell(\alpha) = E(x) + \alpha \cdot g^T \Delta x
$$

This is the tangent line at $\alpha = 0$. The Armijo condition requires the actual energy to lie below the line $E(x) + c \cdot \alpha \cdot g^T \Delta x$, which is a slightly relaxed version of the tangent (since $c \ll 1$).

```
Energy
  |
  |  E(x) + α·g^T·dx        (tangent, slope = g^T·dx < 0)
  | \
  |  \  E(x) + c·α·g^T·dx   (Armijo line, slope = c·g^T·dx)
  |   \ \
  |    \ \
  |     \_\___  E(x + α·dx)  (actual energy)
  |        \
  |         accepted region (below Armijo line)
  +----------------------------------→ α
```

### Why $g^T \Delta x < 0$ (Descent Condition)

For Newton with exact Hessian:

$$
g^T \Delta x = g^T (-H^{-1} g) = -g^T H^{-1} g
$$

If $H$ is positive definite, $g^T H^{-1} g > 0$, so $g^T \Delta x < 0$ — the direction is guaranteed to be a descent direction.

If $g^T \Delta x \ge 0$, the direction is not a descent direction. This can happen when:
- The Hessian is indefinite (not yet PSD-projected)
- Numerical issues corrupt the solve

In this case, the implementation immediately returns failure without probing trial states:

```cpp
if (directional_derivative >= 0.0) {
    return LineSearchResult{.alpha = 0.0, .energy = current_energy, .success = false};
}
```

## Algorithm: Backtracking

The simplest strategy to find an acceptable $\alpha$:

```
α ← α_init (= 1.0)
for i = 0 to max_iterations:
    E_trial ← E(x + α·Δx)
    if E_trial ≤ E(x) + c·α·(g^T·Δx):    // Armijo satisfied
        return (α, E_trial, success=true)
    α ← α · shrink                          // shrink = 0.5
return (α, E_trial, success=false)
```

Each rejection halves the step size. After $k$ rejections, $\alpha = \alpha_{\text{init}} \cdot \text{shrink}^k$:

| Iteration | $\alpha$ |
|-----------|----------|
| 0 | 1.0 |
| 1 | 0.5 |
| 2 | 0.25 |
| 3 | 0.125 |
| ... | ... |
| 20 | $\approx 10^{-6}$ |

### Why Backtracking Always Terminates (for descent directions)

**Theorem**: If $g^T \Delta x < 0$ and $E$ is smooth, then there exists $\bar{\alpha} > 0$ such that the Armijo condition holds for all $\alpha \in (0, \bar{\alpha})$.

**Proof sketch**: By Taylor expansion:

$$
E(x + \alpha \Delta x) = E(x) + \alpha g^T \Delta x + O(\alpha^2)
$$

For small enough $\alpha$, the $O(\alpha^2)$ term is negligible, so:

$$
E(x + \alpha \Delta x) \approx E(x) + \alpha g^T \Delta x < E(x) + c \cdot \alpha g^T \Delta x
$$

The last inequality holds because $g^T \Delta x < 0$ and $c < 1$, so $c \cdot \alpha g^T \Delta x > \alpha g^T \Delta x$ (multiplying a negative number by $c < 1$ makes it less negative).

## API

```cpp
struct LineSearchResult {
    double alpha{0.0};    // accepted step size
    double energy{0.0};   // E(x + α·Δx)
    bool success{false};  // whether Armijo was satisfied
};

using EnergyFunction = std::function<double(double alpha)>;

LineSearchResult backtracking_line_search(
    EnergyFunction energy_fn,          // α → E(x + α·Δx)
    double current_energy,             // E(x)
    double directional_derivative,     // g^T·Δx
    double alpha_init = 1.0,           // initial step size
    double shrink = 0.5,               // backtracking factor
    double armijo_c = 1e-4,            // Armijo constant
    int max_iterations = 20            // max backtracking steps
);
```

## Implementation

### Input Validation

```cpp
if (!std::isfinite(current_energy)) { throw ...; }
if (!std::isfinite(directional_derivative)) { throw ...; }
if (alpha_init <= 0.0) { throw ...; }
if (shrink <= 0.0 || shrink >= 1.0) { throw ...; }
if (armijo_c <= 0.0 || armijo_c >= 1.0) { throw ...; }
```

All parameters must be finite and in their valid ranges.

### Descent Check

```cpp
if (directional_derivative >= 0.0) {
    return LineSearchResult{.alpha = 0.0, .energy = current_energy, .success = false};
}
```

Non-descent directions are rejected immediately. The caller (Newton solver) should handle this by increasing regularization.

### Main Loop

```cpp
double alpha = alpha_init;
double trial_energy = std::numeric_limits<double>::quiet_NaN();
for (int iteration = 0; iteration <= max_iterations; ++iteration) {
    trial_energy = energy_fn(alpha);
    if (trial_energy <= current_energy + armijo_c * alpha * directional_derivative) {
        return LineSearchResult{.alpha = alpha, .energy = trial_energy, .success = true};
    }
    alpha *= shrink;
}
```

Key details:

- `energy_fn(alpha)` evaluates $E(x + \alpha \Delta x)$. The lambda captures `state.x` and `dx` from the Newton solver, constructing `trial_x = state.x + alpha * dx`.
- The Armijo check: `trial_energy <= current_energy + armijo_c * alpha * directional_derivative`. Since `directional_derivative < 0`, the right-hand side is `current_energy - |something positive|`, i.e., it requires the energy to decrease by at least $c \cdot \alpha \cdot |g^T \Delta x|$.
- `alpha *= shrink` halves the step size on each rejection.

### Failure Return

```cpp
return LineSearchResult{.alpha = alpha, .energy = trial_energy, .success = false};
```

If all iterations exhaust without satisfying Armijo, the last `alpha` and `trial_energy` are returned with `success = false`. The Newton solver treats this as a convergence failure.

## Parameter Choices

| Parameter | Default | Rationale |
|-----------|---------|-----------|
| `alpha_init = 1.0` | Full Newton step first. Newton's quadratic convergence only kicks in when $\alpha = 1$ is accepted near the solution. |
| `shrink = 0.5` | Halving is standard. Aggressive enough to find small steps quickly, conservative enough not to skip good steps. |
| `armijo_c = 1e-4` | Very permissive — accepts almost any decrease. Standard in optimization literature (Nocedal & Wright). |
| `max_iterations = 20` | At 20 halvings, $\alpha \approx 10^{-6}$. If this isn't enough, the problem likely has deeper issues. |

## Day 3 Extension: CCD Step Clamping

When contact handling is added, the maximum safe step size $\alpha_{\max}$ will be determined by CCD (Continuous Collision Detection):

$$
\alpha \le \alpha_{\max} = \text{CCD}(x, \Delta x)
$$

The line search will then start from $\min(\alpha_{\text{init}}, \alpha_{\max})$ instead of $\alpha_{\text{init}}$. This only requires changing `alpha_init` at the call site — the line search algorithm itself does not change.
