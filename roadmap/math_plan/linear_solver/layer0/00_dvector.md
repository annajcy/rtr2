# Layer 0 — 动态向量 DVector

## 目标

实现 `DVector<T>` 替换 IPC 中所有 `Eigen::VectorXd` 的使用。这是替换工作中最简单、覆盖面最广的一层。

## 预计工期：2-3 天

## 替换范围

| 当前 Eigen 用法 | 所在文件 | 替换目标 |
|----------------|---------|---------|
| `Eigen::VectorXd x, x_prev, v, mass_diag` | `ipc_state.hpp` | `DVector<Float>` |
| `gradient.dot(gradient)` | `newton_solver.hpp` | `DVector::dot()` |
| `mass.cwiseProduct(dx)` | `inertial_energy.hpp` | `DVector::cwiseProduct()` |
| `gradient.lpNorm<Eigen::Infinity>()` | `newton_solver.hpp` | `DVector::max_abs()` |
| `x.segment<3>(3*i)` | 各 energy 模块 | `DVector::segment3(i)` |
| `x.allFinite()` | `newton_solver.hpp` | `DVector::all_finite()` |

## API 设计

```cpp
namespace pbpt::math {

template <std::floating_point T = Float>
class DVector {
    T* data_;
    size_t size_;
    bool owning_;  // 区分 owned 和 view 模式

public:
    // === 构造 ===
    explicit DVector(size_t n);                    // 未初始化
    static DVector zeros(size_t n);                // 零初始化
    static DVector filled(size_t n, T value);      // 常量填充
    static DVector from_raw(T* ptr, size_t n);     // non-owning view

    DVector(const DVector&);
    DVector(DVector&&) noexcept;
    DVector& operator=(const DVector&);
    DVector& operator=(DVector&&) noexcept;
    ~DVector();

    // === 元素访问 ===
    T& operator[](size_t i);
    const T& operator[](size_t i) const;
    T* data();
    const T* data() const;
    size_t size() const;

    // 关键：per-vertex 3D 访问，返回现有 Vector<T,3> 的引用
    // 这使得 IPC 代码中 x.segment<3>(3*i) 可以无缝替换
    Vector<T, 3>& segment3(size_t vertex_index);
    const Vector<T, 3>& segment3(size_t vertex_index) const;

    // === 算术 ===
    DVector operator+(const DVector& rhs) const;
    DVector operator-(const DVector& rhs) const;
    DVector operator*(T scalar) const;
    DVector operator/(T scalar) const;
    DVector& operator+=(const DVector& rhs);
    DVector& operator-=(const DVector& rhs);
    DVector& operator*=(T scalar);

    // AXPY: this += alpha * x （Newton 更新热路径）
    DVector& axpy(T alpha, const DVector& x);

    // === 归约 ===
    T dot(const DVector& rhs) const;
    T squared_norm() const;
    T norm() const;
    T max_abs() const;                             // L∞ norm
    T sum() const;

    // === 逐元素运算 ===
    DVector cwiseProduct(const DVector& rhs) const;
    DVector cwiseMin(const DVector& rhs) const;
    DVector cwiseMax(const DVector& rhs) const;

    // === 验证 ===
    bool all_finite() const;

    // === 工具 ===
    void set_zero();
    void set_constant(T value);
};

// 标量左乘
template <std::floating_point T>
DVector<T> operator*(T scalar, const DVector<T>& v);

}
```

## 实现要点

### segment3 的零拷贝实现

`segment3(i)` 的核心思路是将 `data_ + 3*i` 直接 reinterpret 为 `Vector<T,3>&`。前提条件：

- `Vector<T,3>` 的内存布局必须是 3 个连续的 `T`（当前 `pbpt::math::Vector` 继承自 `Tuple`，满足此条件）
- 需要 `static_assert(sizeof(Vector<T,3>) == 3 * sizeof(T))` 确保无 padding

```cpp
Vector<T, 3>& segment3(size_t vertex_index) {
    static_assert(sizeof(Vector<T, 3>) == 3 * sizeof(T),
                  "Vector<T,3> must be tightly packed");
    return *reinterpret_cast<Vector<T, 3>*>(data_ + 3 * vertex_index);
}
```

### SIMD 友好的内存布局

- 内部 `data_` 分配时对齐到 64 字节（cache line + AVX-512 友好）
- 归约操作（`dot`, `norm`, `sum`）后续可用 SIMD 加速，初版先用标量循环

### 过渡阶段的 Eigen 互操作

Layer 0 完成后，Newton solver 内部仍然使用 Eigen 做稀疏求解。需要临时的转换函数：

```cpp
// 临时工具，Layer 2 完成后删除
Eigen::Map<Eigen::VectorXd> to_eigen_map(DVector<double>& v) {
    return Eigen::Map<Eigen::VectorXd>(v.data(), v.size());
}
```

## File-by-File TODO

### 1. `external/pbpt/src/pbpt/math/dynamic/dvector.hpp` — 新建

- [ ] 实现 DVector 类，含所有上述 API
- [ ] `static_assert` 验证 `Vector<T,3>` 布局
- [ ] 64 字节对齐分配（`std::aligned_alloc` 或平台等价物）
- [ ] 单元测试：算术、归约、segment3、边界条件

### 2. `src/rtr/system/physics/ipc/ipc_state.hpp` — 修改

- [ ] `#include <Eigen/Core>` → `#include <pbpt/math/dynamic/dvector.hpp>`
- [ ] `Eigen::VectorXd` → `pbpt::math::DVector<pbpt::math::Float>`
- [ ] `.segment<3>(3*i)` → `.segment3(i)`

### 3. `src/rtr/system/physics/ipc/energy/inertial_energy.hpp` — 修改

- [ ] 替换 `cwiseProduct` 调用
- [ ] 替换 `squaredNorm` → `squared_norm()`

### 4. `src/rtr/system/physics/ipc/energy/gravity_energy.hpp` — 修改

- [ ] 替换梯度向量的逐元素访问

### 5. `src/rtr/system/physics/ipc/solver/newton_solver.hpp` — 部分修改

- [ ] 梯度和搜索方向改为 `DVector`
- [ ] `lpNorm<Infinity>()` → `max_abs()`
- [ ] 保留 Eigen 稀疏求解，添加临时 `to_eigen_map()` 转换

## 验证方案

1. **单元测试**：`DVector` 的每个 API 独立测试
2. **精度对比**：同一 IPC 场景运行 100 帧，逐帧对比替换前后的位置向量，diff < 1e-12
3. **编译检查**：确认 Layer 0 之后只剩下 `<Eigen/SparseCore>` 和 `<Eigen/SparseCholesky>` 的 include

## 风险与缓解

| 风险 | 缓解策略 |
|------|---------|
| `Vector<T,3>` 有 padding 导致 `segment3` UB | `static_assert` 编译期拦截 |
| 归约精度与 Eigen 有微小差异 | 使用 Kahan summation 或 pairwise summation |
| 过渡期 Eigen 互操作引入额外拷贝 | `to_eigen_map()` 是零拷贝映射 |
