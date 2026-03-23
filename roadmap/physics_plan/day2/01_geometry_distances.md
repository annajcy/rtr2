# Phase 1: 几何距离模块

## 目标

实现 point-triangle 和 edge-edge 的距离平方函数及其梯度、Hessian。这是 barrier potential 和 CCD 的数学地基。

## 为什么用距离平方而不是距离

barrier potential 的输入是 $d^2$（距离平方），而不是 $d$：

1. $d^2$ 是光滑的（$d = \sqrt{d^2}$ 在 $d=0$ 处不可微）
2. $d^2$ 的梯度和 Hessian 不需要除以 $d$，避免数值不稳定
3. IPC 原始论文就是用 $d^2$ 作为 barrier 的输入

## 距离类型分类

IPC 中所有原始接触对只有两种：

| 类型 | 几何原语 | DOF 数 | 用途 |
|------|---------|--------|------|
| Point-Triangle (PT) | 1 点 + 3 三角形顶点 | 12 DOF | 点对面的距离 |
| Edge-Edge (EE) | 2 边端点 × 2 | 12 DOF | 棱对棱的距离 |

Vertex-Vertex 和 Point-Edge 是退化情况，通过 PT 和 EE 的参数分类自动覆盖。

---

## Point-Triangle 距离

### 数学定义

给定点 $p$ 和三角形 $(t_0, t_1, t_2)$，最近距离取决于投影点落在三角形的哪个区域。

将投影参数化：$q = t_0 + s(t_1 - t_0) + t(t_2 - t_0)$，最近点满足不同约束：

| 区域 | 条件 | 距离退化为 |
|------|------|-----------|
| 三角形内部 | $s \geq 0, t \geq 0, s+t \leq 1$ | Point-Plane distance |
| 边 $t_0 t_1$ | $t = 0, 0 \leq s \leq 1$ | Point-Edge distance |
| 边 $t_0 t_2$ | $s = 0, 0 \leq t \leq 1$ | Point-Edge distance |
| 边 $t_1 t_2$ | $s + t = 1, s \geq 0$ | Point-Edge distance |
| 顶点 $t_0$ | $s = 0, t = 0$ | Point-Point distance |
| 顶点 $t_1$ | $s = 1, t = 0$ | Point-Point distance |
| 顶点 $t_2$ | $s = 0, t = 1$ | Point-Point distance |

### 实现策略

不要写 7 个 case 的 if-else 树。推荐分层实现：

```
point_point_distance_squared(p0, p1) -> double, gradient(6), Hessian(6x6)
point_edge_distance_squared(p, e0, e1) -> double, gradient(9), Hessian(9x9)
point_triangle_distance_squared(p, t0, t1, t2) -> double, gradient(12), Hessian(12x12)
```

其中 `point_triangle_distance_squared` 内部：
1. 计算投影参数 $(s, t)$
2. 分类到 7 个区域之一
3. 调用对应的子函数（PP / PE / PT-interior）
4. 返回局部 12-DOF 梯度和 Hessian

### 局部 DOF 排布

对 PT，12 个局部 DOF 排列为：

```
[p_x, p_y, p_z, t0_x, t0_y, t0_z, t1_x, t1_y, t1_z, t2_x, t2_y, t2_z]
```

### 核心公式（PT-interior case）

投影到三角形平面的距离：

$$d^2_{PT} = ((p - t_0) \cdot n)^2$$

其中 $n = \frac{(t_1 - t_0) \times (t_2 - t_0)}{|(t_1 - t_0) \times (t_2 - t_0)|}$

但注意：这个公式只在投影点落在三角形内部时正确。其他情况退化为 PE 或 PP。

实现时推荐使用向量叉积的 squared norm 形式，避免归一化带来的额外导数复杂度。

---

## Edge-Edge 距离

### 数学定义

给定两条边 $(e_{a0}, e_{a1})$ 和 $(e_{b0}, e_{b1})$，类似地根据最近点参数 $(s, t)$ 分类：

| 区域 | 退化为 |
|------|--------|
| 两条边内部 | 真正的 EE distance |
| $s$ 或 $t$ 在端点 | PE 或 PP distance |

### 平行边退化

当两条边近似平行时，EE 距离公式中的分母趋近于零。必须做退化检测：

```cpp
const double cross_norm_sq = cross.squaredNorm();
if (cross_norm_sq < kParallelThreshold) {
    // 退化为 point-edge 距离的最小值
}
```

推荐 `kParallelThreshold = 1e-20`。

### 局部 DOF 排布

```
[ea0_x, ea0_y, ea0_z, ea1_x, ea1_y, ea1_z, eb0_x, eb0_y, eb0_z, eb1_x, eb1_y, eb1_z]
```

---

## 公共辅助函数

建议在 `geometry/distance_utils.hpp` 中放：

```cpp
// Point-Point 距离平方 + 梯度 + Hessian
struct PPDistanceResult {
    double distance_squared;
    Eigen::Vector<double, 6> gradient;
    Eigen::Matrix<double, 6, 6> hessian;
};
PPDistanceResult point_point_distance(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1);

// Point-Edge 距离平方 + 梯度 + Hessian
struct PEDistanceResult {
    double distance_squared;
    Eigen::Vector<double, 9> gradient;
    Eigen::Matrix<double, 9, 9> hessian;
};
PEDistanceResult point_edge_distance(const Eigen::Vector3d& p, const Eigen::Vector3d& e0, const Eigen::Vector3d& e1);
```

---

## 导数计算策略

### 推荐：解析导数

每个距离函数都需要解析的 gradient 和 Hessian。不要用自动微分或有限差分作为运行时实现——只用有限差分做测试验证。

### 链式法则结构

以 PT-interior 为例，距离平方对 12 DOF 的梯度：

$$\frac{\partial d^2}{\partial x} = \frac{\partial d^2}{\partial \text{local\_coords}} \cdot \frac{\partial \text{local\_coords}}{\partial x}$$

其中 `local_coords` 是 $(p, t_0, t_1, t_2)$ 的展开。

### Hessian 的 SPD 投影

距离的 Hessian 不一定是半正定的。barrier 的 Hessian 需要通过链式法则组合距离 Hessian 和 barrier 导数，结果需要做 PSD projection。

推荐在 barrier potential 层统一做 PSD projection（而不是在距离层做），因为：
1. 距离本身的 Hessian 没有 SPD 要求
2. barrier Hessian = barrier'' * grad_d * grad_d^T + barrier' * Hessian_d，只有组合后的结果需要 SPD

---

## 输出文件

| 文件 | 内容 |
|------|------|
| `src/rtr/system/physics/ipc/geometry/distance_type.hpp` | PT/EE/PE/PP 类型枚举 |
| `src/rtr/system/physics/ipc/geometry/point_point_distance.hpp` | PP 距离 + 梯度 + Hessian |
| `src/rtr/system/physics/ipc/geometry/point_edge_distance.hpp` | PE 距离 + 梯度 + Hessian |
| `src/rtr/system/physics/ipc/geometry/point_triangle_distance.hpp` | PT 距离 + 梯度 + Hessian（含分类） |
| `src/rtr/system/physics/ipc/geometry/edge_edge_distance.hpp` | EE 距离 + 梯度 + Hessian（含平行退化） |

## 强制测试

每个距离函数都必须通过有限差分检查：

| 测试 | 方法 |
|------|------|
| PP gradient | $\frac{f(x+h e_i) - f(x-h e_i)}{2h}$ vs 解析梯度 |
| PP Hessian | 有限差分 Jacobian of gradient vs 解析 Hessian |
| PE gradient + Hessian | 同上 |
| PT gradient + Hessian | 同上，测试所有 7 个区域 |
| EE gradient + Hessian | 同上，包括平行退化 case |

推荐 $h = 10^{-7}$，相对误差阈值 $10^{-5}$。

测试文件：

| 文件 | 内容 |
|------|------|
| `test/system/physics/ipc/geometry/point_point_distance_test.cpp` | PP fd check |
| `test/system/physics/ipc/geometry/point_edge_distance_test.cpp` | PE fd check |
| `test/system/physics/ipc/geometry/point_triangle_distance_test.cpp` | PT fd check（多区域） |
| `test/system/physics/ipc/geometry/edge_edge_distance_test.cpp` | EE fd check（含平行） |
