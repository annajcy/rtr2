# `line_search.hpp`

`src/rtr/system/physics/ipc/solver/line_search.hpp` 实现了 IPC Newton 求解器使用的回溯 Armijo line search。

## 为什么需要 Line Search

Newton 法通过求解 $H \Delta x = -g$ 给出搜索方向 $\Delta x$。但整步 $x + \Delta x$ 可能过冲，尤其在：

- Hessian 只在局部准确（二次模型在远离 $x^k$ 处失效）
- 能量面高度非线性（大变形）
- Hessian 被正则化或投影过（方向是近似的）

Line search 缩放步长：$x^{k+1} = x^k + \alpha \Delta x$，选择 $\alpha \in (0, 1]$ 确保**充分的能量下降**。

## 理论：Armijo 条件

### 充分下降准则

朴素的检查 "$E(x + \alpha \Delta x) < E(x)$" 太弱——它会接受极小的下降量，导致收敛停滞。**Armijo 条件**（也叫充分下降条件，或第一 Wolfe 条件）要求：

$$
\boxed{E(x + \alpha \Delta x) \le E(x) + c \cdot \alpha \cdot g^T \Delta x}
$$

其中：

| 符号 | 含义 | 取值 |
|------|------|------|
| $E(x)$ | 当前能量 | — |
| $\alpha$ | 步长 | $\in (0, 1]$ |
| $\Delta x$ | Newton 方向 | $H^{-1}(-g)$ |
| $g$ | 梯度 $\nabla E(x)$ | — |
| $c$ | Armijo 常数 | $10^{-4}$ |
| $g^T \Delta x$ | 方向导数 | 必须 $< 0$ |

### 几何解释

在 $\alpha = 0$ 处，能量沿搜索方向的线性模型为：

$$
\ell(\alpha) = E(x) + \alpha \cdot g^T \Delta x
$$

这是 $\alpha = 0$ 处的切线。Armijo 条件要求实际能量位于 $E(x) + c \cdot \alpha \cdot g^T \Delta x$ 这条线之下，这是切线的略微放松版本（因为 $c \ll 1$）。

```
Energy
  |
  |  E(x) + α·g^T·dx        （切线，斜率 = g^T·dx < 0）
  | \
  |  \  E(x) + c·α·g^T·dx   （Armijo 线，斜率 = c·g^T·dx）
  |   \ \
  |    \ \
  |     \_\___  E(x + α·dx)  （实际能量）
  |        \
  |         接受区域（Armijo 线以下）
  +----------------------------------→ α
```

### 为什么要求 $g^T \Delta x < 0$（下降条件）

对于精确 Hessian 的 Newton 法：

$$
g^T \Delta x = g^T (-H^{-1} g) = -g^T H^{-1} g
$$

如果 $H$ 正定，$g^T H^{-1} g > 0$，因此 $g^T \Delta x < 0$——方向保证是下降方向。

如果 $g^T \Delta x \ge 0$，方向不是下降方向。这在以下情况可能发生：
- Hessian 不定（尚未 PSD 投影）
- 数值问题污染了求解

此时实现立即返回失败，不再探测 trial 状态：

```cpp
if (directional_derivative >= 0.0) {
    return LineSearchResult{.alpha = 0.0, .energy = current_energy, .success = false};
}
```

## 算法：回溯

寻找可接受 $\alpha$ 的最简策略：

```
α ← α_init (= 1.0)
for i = 0 to max_iterations:
    E_trial ← E(x + α·Δx)
    if E_trial ≤ E(x) + c·α·(g^T·Δx):    // Armijo 满足
        return (α, E_trial, success=true)
    α ← α · shrink                          // shrink = 0.5
return (α, E_trial, success=false)
```

每次拒绝后步长减半。经过 $k$ 次拒绝后，$\alpha = \alpha_{\text{init}} \cdot \text{shrink}^k$：

| 迭代 | $\alpha$ |
|------|----------|
| 0 | 1.0 |
| 1 | 0.5 |
| 2 | 0.25 |
| 3 | 0.125 |
| ... | ... |
| 20 | $\approx 10^{-6}$ |

### 为什么回溯一定终止（对下降方向）

**定理**：若 $g^T \Delta x < 0$ 且 $E$ 光滑，则存在 $\bar{\alpha} > 0$ 使得 Armijo 条件对所有 $\alpha \in (0, \bar{\alpha})$ 成立。

**证明概要**：Taylor 展开：

$$
E(x + \alpha \Delta x) = E(x) + \alpha g^T \Delta x + O(\alpha^2)
$$

当 $\alpha$ 足够小时，$O(\alpha^2)$ 可忽略，于是：

$$
E(x + \alpha \Delta x) \approx E(x) + \alpha g^T \Delta x < E(x) + c \cdot \alpha g^T \Delta x
$$

最后一个不等式成立是因为 $g^T \Delta x < 0$ 且 $c < 1$，所以 $c \cdot \alpha g^T \Delta x > \alpha g^T \Delta x$（负数乘以 $c < 1$ 会变得"不那么负"）。

## 接口

```cpp
struct LineSearchResult {
    double alpha{0.0};    // 接受的步长
    double energy{0.0};   // E(x + α·Δx)
    bool success{false};  // Armijo 是否满足
};

using EnergyFunction = std::function<double(double alpha)>;

LineSearchResult backtracking_line_search(
    EnergyFunction energy_fn,          // α → E(x + α·Δx)
    double current_energy,             // E(x)
    double directional_derivative,     // g^T·Δx
    double alpha_init = 1.0,           // 初始步长
    double shrink = 0.5,               // 回溯因子
    double armijo_c = 1e-4,            // Armijo 常数
    int max_iterations = 20            // 最大回溯次数
);
```

## 实现

### 输入验证

```cpp
if (!std::isfinite(current_energy)) { throw ...; }
if (!std::isfinite(directional_derivative)) { throw ...; }
if (alpha_init <= 0.0) { throw ...; }
if (shrink <= 0.0 || shrink >= 1.0) { throw ...; }
if (armijo_c <= 0.0 || armijo_c >= 1.0) { throw ...; }
```

所有参数必须有限且在有效范围内。

### 下降方向检查

```cpp
if (directional_derivative >= 0.0) {
    return LineSearchResult{.alpha = 0.0, .energy = current_energy, .success = false};
}
```

非下降方向立即拒绝。调用方（Newton solver）应通过增大 regularization 来处理。

### 主循环

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

关键细节：

- `energy_fn(alpha)` 评估 $E(x + \alpha \Delta x)$。lambda 从 Newton solver 捕获 `state.x` 和 `dx`，构造 `trial_x = state.x + alpha * dx`。
- Armijo 检查：`trial_energy <= current_energy + armijo_c * alpha * directional_derivative`。由于 `directional_derivative < 0`，右侧为 `current_energy - |正值|`，即要求能量至少下降 $c \cdot \alpha \cdot |g^T \Delta x|$。
- `alpha *= shrink` 每次拒绝后步长减半。

### 失败返回

```cpp
return LineSearchResult{.alpha = alpha, .energy = trial_energy, .success = false};
```

如果所有迭代用尽仍未满足 Armijo，返回最后的 `alpha` 和 `trial_energy`，`success = false`。Newton solver 将此视为收敛失败。

## 参数选择

| 参数 | 默认值 | 理由 |
|------|--------|------|
| `alpha_init = 1.0` | 先尝试完整 Newton 步。只有 $\alpha = 1$ 被接受时，Newton 法的二次收敛才能生效。 |
| `shrink = 0.5` | 减半是标准选择。足够激进以快速找到小步长，又足够保守不会跳过好的步长。 |
| `armijo_c = 1e-4` | 非常宽松——几乎接受任何下降。这是优化文献中的标准值（Nocedal & Wright）。 |
| `max_iterations = 20` | 20 次减半后 $\alpha \approx 10^{-6}$。如果这还不够，问题很可能有更深层的问题。 |

## Day 3 扩展：CCD 步长裁剪

加入碰撞处理后，最大安全步长 $\alpha_{\max}$ 由 CCD（连续碰撞检测）决定：

$$
\alpha \le \alpha_{\max} = \text{CCD}(x, \Delta x)
$$

Line search 将从 $\min(\alpha_{\text{init}}, \alpha_{\max})$ 开始而非 $\alpha_{\text{init}}$。这只需要在调用点修改 `alpha_init`——line search 算法本身不需要改变。
