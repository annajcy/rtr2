# Layer 1 — 3x3 SVD + 解析距离导数

## 目标

替换 IPC 中两个最特化的 Eigen 功能：

1. `Eigen::JacobiSVD<Matrix3d>` → 自研 `svd3x3()` 专用分解
2. `Eigen::AutoDiffScalar` 二阶自动微分 → 手推解析 gradient + Hessian

## 预计工期：3-4 天

---

## Part A：3x3 专用 SVD（1-2 天）

### 替换范围

| 当前 Eigen 用法 | 所在文件 |
|----------------|---------|
| `Eigen::JacobiSVD<Eigen::Matrix3d> svd(F, ComputeFullU \| ComputeFullV)` | `tet_fixed_corotated.hpp` |
| `svd.matrixU()`, `svd.matrixV()`, `svd.singularValues()` | `tet_fixed_corotated.hpp` |
| 符号修正：`det(U) < 0` 时翻转最后一列 | `tet_fixed_corotated.hpp` |

### 算法选型：McAdams et al. 2011

参考论文：*Computing the Singular Value Decomposition of 3x3 Matrices with Minimal Branching and Elementary Floating Point Operations* (McAdams, Selle, Tamstorf, Teran, Sifakis, 2011)

核心思路：

1. **对称特征值分解**：先算 `F^T * F` 的特征值分解 → 得到 `V` 和 `Σ²`
2. **Givens 旋转迭代**：用 Jacobi 迭代（Givens QR）对称矩阵特征分解，每次消去一个非对角元素
3. **U 恢复**：`U = F * V * Σ^{-1}`（处理零奇异值的退化情况）
4. **符号修正**：确保 `det(U) > 0` 和 `det(V) > 0`

为什么选这个算法：

- 专为 3x3 设计，无动态分配
- 分支极少，SIMD 友好（后续可 vectorize）
- GPU 友好（无递归，无内存分配）
- 在 FEM 场景下大量 benchmark 验证过

### API 设计

```cpp
namespace pbpt::math {

template <std::floating_point T>
struct SVDResult3 {
    Matrix<T, 3, 3> U;
    Vector<T, 3> sigma;      // 奇异值，降序排列
    Matrix<T, 3, 3> V;
};

// 完整 SVD：F = U * diag(sigma) * V^T
template <std::floating_point T>
SVDResult3<T> svd3x3(const Matrix<T, 3, 3>& F);

// 极分解：F = R * S，直接返回旋转 R
// 这是 corotated 材料的热路径，避免不必要的中间构造
template <std::floating_point T>
Matrix<T, 3, 3> polar_rotation(const Matrix<T, 3, 3>& F);

// 带符号修正的极分解（处理 det(F) < 0 的翻转情况）
template <std::floating_point T>
Matrix<T, 3, 3> polar_rotation_signed(const Matrix<T, 3, 3>& F);

}
```

### 实现要点

#### Givens 旋转的 3x3 特化

通用 Jacobi 迭代选最大非对角元素消去，但对 3x3 只有 3 个非对角位置 `(0,1), (0,2), (1,2)`，可以展开为固定的 sweep 循环：

```
repeat 4 times:  // 通常 3-4 次即收敛
    givens_rotation(S, 0, 1)
    givens_rotation(S, 0, 2)
    givens_rotation(S, 1, 2)
```

每次 Givens 旋转只涉及 2x2 子问题，可以用解析公式直接算角度。

#### 退化情况处理

- 奇异值为零：`sigma[i] < epsilon` 时，对应的 `U` 列通过叉积补全
- `det(F) < 0`（翻转四面体）：将最小奇异值取反，对应 `U` 列取反

### File-by-File TODO

#### 1. `external/pbpt/src/pbpt/math/decomposition/svd3x3.hpp` — 新建

- [ ] 实现 `svd3x3()` — McAdams 2011 算法
- [ ] 实现 `polar_rotation()` 和 `polar_rotation_signed()`
- [ ] 处理退化情况（零奇异值、负行列式）
- [ ] 收敛次数硬编码为 4 次 sweep（3x3 已足够）

#### 2. `src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp` — 修改

- [ ] `#include <Eigen/SVD>` → `#include <pbpt/math/decomposition/svd3x3.hpp>`
- [ ] `Eigen::JacobiSVD<Eigen::Matrix3d> svd(...)` → `auto [U, sigma, V] = svd3x3(F)`
- [ ] 移除手动符号修正代码，改用 `polar_rotation_signed()`

### 验证方案

- 随机生成 10000 个 3x3 矩阵（含正常、近奇异、负行列式情况）
- 对比 Eigen `JacobiSVD` 结果：`||U_ours - U_eigen|| < 1e-10`
- 重建验证：`||F - U * diag(sigma) * V^T|| < 1e-12`
- 正交性验证：`||U^T * U - I|| < 1e-14`，`||V^T * V - I|| < 1e-14`

---

## Part B：解析距离导数（2-3 天）

### 替换范围

| 距离类型 | DOF 数 | 当前实现 | 所在文件 |
|---------|--------|---------|---------|
| Point-Point | 6 | AutoDiff 二阶 | `point_point_distance.hpp` |
| Point-Edge | 9 | AutoDiff 二阶 | `point_edge_distance.hpp` |
| Point-Triangle | 12 | AutoDiff 二阶 + 区域分类 | `point_triangle_distance.hpp` |
| Edge-Edge | 12 | AutoDiff 二阶 + 平行处理 | `edge_edge_distance.hpp` |

### 为什么用解析导数替代 AutoDiff

- IPC 的距离函数类型是固定的（只有上述 4 种），不需要通用 AutoDiff 的灵活性
- 解析公式在 IPC 文献中已有完善推导（Li et al. 2020, 2021）
- 消除 AutoDiff 的嵌套模板实例化，编译速度大幅提升
- 运行时无 AutoDiff 的额外存储和运算开销

### 每种距离的解析公式概览

#### Point-Point Distance

输入：`p0, p1` (各 3D)，6 DOF

```
d² = ||p0 - p1||²
∇d² = 2 * [p0 - p1, p1 - p0]  (6×1)
H(d²) = 2 * [[I, -I], [-I, I]]  (6×6，常量矩阵)
```

最简单的情况，Hessian 是常量。

#### Point-Edge Distance

输入：`p, e0, e1` (各 3D)，9 DOF

需要计算点到线段的最近点参数 `t = clamp((p-e0)·(e1-e0) / ||e1-e0||², 0, 1)`，然后根据 `t` 所在区域（端点或内部）选择不同的 gradient/Hessian 公式。

#### Point-Triangle Distance

输入：`p, t0, t1, t2` (各 3D)，12 DOF

最复杂的情况。需要先分类最近特征（vertex / edge / face），然后对应不同的导数公式。面投影情况下：

```
d² = ((p - t0) · n)²   where n = normalize((t1-t0) × (t2-t0))
```

梯度和 Hessian 涉及法线对顶点的微分。

#### Edge-Edge Distance

输入：`ea0, ea1, eb0, eb1` (各 3D)，12 DOF

需要处理平行边的退化情况。非平行时为两条线段间最近点距离的平方。

### 实现策略

每种距离函数实现三个函数：

```cpp
namespace pbpt::math {

// Point-Point 示例，其他类似
template <std::floating_point T>
T point_point_distance_squared(
    const Vector<T,3>& p0, const Vector<T,3>& p1);

template <std::floating_point T>
void point_point_distance_gradient(
    const Vector<T,3>& p0, const Vector<T,3>& p1,
    Vector<T,6>& gradient);

template <std::floating_point T>
void point_point_distance_hessian(
    const Vector<T,3>& p0, const Vector<T,3>& p1,
    Matrix<T,6,6>& hessian);

}
```

### File-by-File TODO

#### 1. `external/pbpt/src/pbpt/math/spatial/vector.hpp` — 可能需要扩展

- [ ] 确认 `Vector<T, 6>`, `Vector<T, 9>`, `Vector<T, 12>` 可用（应已支持，需验证）

#### 2. `src/rtr/system/physics/ipc/geometry/distance/point_point_distance.hpp` — 重写

- [ ] 移除 AutoDiff 实现
- [ ] 实现解析 value / gradient / hessian

#### 3. `src/rtr/system/physics/ipc/geometry/distance/point_edge_distance.hpp` — 重写

- [ ] 实现区域分类（端点 vs 内部）
- [ ] 各区域的解析 gradient / hessian

#### 4. `src/rtr/system/physics/ipc/geometry/distance/point_triangle_distance.hpp` — 重写

- [ ] 实现 Voronoi 区域分类（vertex / edge / face）
- [ ] 面投影情况的法线微分
- [ ] 各区域的解析 gradient / hessian

#### 5. `src/rtr/system/physics/ipc/geometry/distance/edge_edge_distance.hpp` — 重写

- [ ] 实现平行检测和退化处理
- [ ] 非平行情况的解析 gradient / hessian

#### 6. `src/rtr/system/physics/ipc/geometry/distance/distance_common.hpp` — 修改

- [ ] 移除 `#include <unsupported/Eigen/AutoDiff>`
- [ ] 移除 AutoDiff 类型定义（`FirstDerivative`, `SecondDerivative` 等）

### 验证方案

对每种距离类型：

1. **对比 AutoDiff**：随机采样 10000 组输入（含退化情况），解析结果 vs AutoDiff 结果 diff < 1e-10
2. **有限差分验证**：gradient 用 `(f(x+h) - f(x-h)) / 2h` 验证，hessian 用 gradient 的有限差分验证
3. **退化鲁棒性**：共线点、共面点、零长度边、平行边等特殊情况不崩溃

## 风险与缓解

| 风险 | 缓解策略 |
|------|---------|
| Point-Triangle 区域分类与原 AutoDiff 实现不一致 | 用相同的区域分类逻辑，只替换导数计算 |
| Edge-Edge 平行退化导致数值不稳定 | 平行情况退化到 Point-Edge，设合理阈值 |
| SVD 收敛不够快 | McAdams 证明 4 次 sweep 对 3x3 已经足够，加 assert 监测 |
