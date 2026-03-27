# Layer 2 — 稀疏矩阵 + 求解器后端

## 目标

替换 IPC Newton solver 中的稀疏线性代数：

1. `Eigen::Triplet<double>` → `pbpt::math::Triplet<T>`
2. `Eigen::SparseMatrix<double>` → `pbpt::math::SparseMatrix<T>` (CSR)
3. `Eigen::SimplicialLDLT` → `pbpt::math::LinearSolver<T, Backend>`

这是整个替换中最核心、风险最高的一层。

## 预计工期：4-5 天

---

## Part A：稀疏矩阵存储（1-2 天）

### API 设计

```cpp
namespace pbpt::math {

// === COO 格式条目 ===
template <std::floating_point T = Float>
struct Triplet {
    size_t row;
    size_t col;
    T value;

    Triplet() = default;
    Triplet(size_t r, size_t c, T v) : row(r), col(c), value(v) {}
};

// === CSR 稀疏矩阵 ===
template <std::floating_point T = Float>
class SparseMatrix {
    std::vector<size_t> row_ptr_;    // 长度 rows+1
    std::vector<size_t> col_idx_;    // 长度 nnz
    std::vector<T> values_;          // 长度 nnz
    size_t rows_, cols_;

public:
    // --- 构建 ---

    // 从 triplet 列表构建，重复项累加（等价于 Eigen::setFromTriplets）
    static SparseMatrix from_triplets(
        size_t rows, size_t cols,
        std::span<const Triplet<T>> triplets);

    // --- 查询 ---
    size_t rows() const;
    size_t cols() const;
    size_t nnz() const;

    // --- 运算 ---

    // SpMV: y = A * x
    void multiply(const DVector<T>& x, DVector<T>& y) const;

    // y = A * x + beta * y
    void multiply_add(const DVector<T>& x, DVector<T>& y, T beta) const;

    // --- 对角线操作（Newton solver 正则化） ---

    // H += lambda * I
    void add_scalar_diagonal(T lambda);

    // H += diag(d)
    void add_diagonal(const DVector<T>& d);

    // --- 子矩阵（free DOF 提取） ---

    // 提取 indices 指定的行列子矩阵
    SparseMatrix extract_submatrix(std::span<const size_t> indices) const;

    // --- 原始数据访问（后端求解器消费） ---
    const size_t* row_ptr_data() const;
    const size_t* col_idx_data() const;
    const T* values_data() const;
    T* values_data();  // 可变版本，用于原地修改 values

    // --- 结构复用 ---
    // 保留符号结构，只清零数值（Newton 迭代中结构不变时复用）
    void zero_values();

    // 判断两个矩阵是否具有相同的非零结构
    bool same_structure(const SparseMatrix& other) const;
};

}
```

### 实现要点

#### from_triplets 的高效实现

1. 按行排序 triplets（`std::sort` by row, then col）
2. 合并同行同列的重复项（累加 value）
3. 一次遍历构建 `row_ptr_`, `col_idx_`, `values_`

对于 IPC 典型规模（10K-100K DOF），triplet 排序是主要开销。可用 counting sort（桶排序）按行分桶，然后每桶内按列排序，复杂度 O(nnz + rows)。

#### CSR 格式选择理由

- SuiteSparse CHOLMOD 原生接受 CSC（CSR 的转置），转换代价低
- MKL Pardiso 原生接受 CSR
- cuSPARSE 原生接受 CSR
- SpMV 行遍历对 cache 友好

#### 子矩阵提取

Newton solver 当前的做法是从全局 Hessian 中提取 free DOF 对应的子矩阵。`extract_submatrix` 的实现：

1. 构建 `global_to_local` 映射表
2. 遍历原矩阵，保留两个索引都在 indices 中的条目
3. 重映射索引并构建新 CSR

### File-by-File TODO

#### 1. `external/pbpt/src/pbpt/math/sparse/triplet.hpp` — 新建

- [ ] `Triplet<T>` 结构体

#### 2. `external/pbpt/src/pbpt/math/sparse/sparse_matrix.hpp` — 新建

- [ ] CSR 存储实现
- [ ] `from_triplets()` 含重复项累加
- [ ] `multiply()` SpMV
- [ ] `add_scalar_diagonal()`, `add_diagonal()`
- [ ] `extract_submatrix()`

---

## Part B：线性求解器接口 + 后端（2-3 天）

### 架构设计

```
LinearSolver<T, Backend>
    │
    ├── SuiteSparseBackend    ← 主力后端（CHOLMOD LDLT）
    ├── EigenBackend          ← 过渡后端（SimplicialLDLT wrapper）
    └── MklPardisoBackend     ← 可选后端（Intel 平台高性能）
```

### 求解器接口

```cpp
namespace pbpt::math {

enum class SolverStatus {
    Success,
    NumericalIssue,   // 分解失败（矩阵不正定等）
    InvalidInput       // 维度不匹配等
};

// Backend 需要满足的 concept
template <typename B, typename T>
concept SolverBackend = requires(B b, const SparseMatrix<T>& A,
                                  const DVector<T>& rhs, DVector<T>& sol) {
    { b.analyze(A) } -> std::same_as<SolverStatus>;
    { b.factorize(A) } -> std::same_as<SolverStatus>;
    { b.solve(rhs, sol) } -> std::same_as<SolverStatus>;
};

template <std::floating_point T, SolverBackend<T> Backend>
class LinearSolver {
    Backend backend_;

public:
    // 符号分析（确定非零结构，只需做一次）
    SolverStatus analyze(const SparseMatrix<T>& A);

    // 数值分解（每次 Hessian 更新后调用）
    SolverStatus factorize(const SparseMatrix<T>& A);

    // 求解 Ax = b（分解完成后可多次调用）
    SolverStatus solve(const DVector<T>& b, DVector<T>& x);

    // 便捷：analyze + factorize + solve
    SolverStatus compute_and_solve(
        const SparseMatrix<T>& A,
        const DVector<T>& b,
        DVector<T>& x);
};

}
```

### SuiteSparse 后端实现

```cpp
namespace pbpt::math {

class SuiteSparseBackend {
    cholmod_common common_;
    cholmod_factor* factor_ = nullptr;
    bool analyzed_ = false;

public:
    SuiteSparseBackend();
    ~SuiteSparseBackend();

    SolverStatus analyze(const SparseMatrix<double>& A);
    SolverStatus factorize(const SparseMatrix<double>& A);
    SolverStatus solve(const DVector<double>& b, DVector<double>& x);

private:
    // CSR → cholmod_sparse 的零拷贝适配
    // CHOLMOD 使用 CSC 格式，但对称矩阵 CSR = CSC^T = CSC
    cholmod_sparse to_cholmod(const SparseMatrix<double>& A);
};

}
```

**关键性能点：**

- `analyze()` 做符号分解（fill-reducing ordering），Newton 迭代中只需做一次
- `factorize()` 做数值分解，每次 Hessian 更新后调用
- 对称正定矩阵的 CSR 和 CSC 等价（只存上/下三角），可零拷贝传入 CHOLMOD

### Eigen 过渡后端

```cpp
namespace pbpt::math {

class EigenBackend {
    // 内部持有 Eigen::SimplicialLDLT
    // 接受 pbpt::math::SparseMatrix，内部转换为 Eigen::SparseMatrix

    SolverStatus analyze(const SparseMatrix<double>& A);
    SolverStatus factorize(const SparseMatrix<double>& A);
    SolverStatus solve(const DVector<double>& b, DVector<double>& x);

private:
    // CSR → Eigen::SparseMatrix 的转换（有拷贝开销，过渡用）
    Eigen::SparseMatrix<double> to_eigen(const SparseMatrix<double>& A);
};

}
```

这个后端的唯一作用是 **安全回退**：如果 SuiteSparse 后端出问题，可以切换到 Eigen 后端对比排查。Layer 3 完成后删除。

### Newton Solver 集成

替换 `newton_solver.hpp` 中的求解流程：

```cpp
// 替换前
Eigen::SparseMatrix<double> H(n, n);
H.setFromTriplets(triplets.begin(), triplets.end());
Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
solver.compute(H);
Eigen::VectorXd dx = solver.solve(-gradient);

// 替换后
auto H = SparseMatrix<double>::from_triplets(n, n, triplets);
LinearSolver<double, SuiteSparseBackend> solver;
solver.analyze(H);        // 首次迭代
solver.factorize(H);      // 每次迭代
solver.solve(neg_gradient, dx);
```

**优化点：** `analyze()` 只在第一次 Newton 迭代时调用（Hessian 的非零结构在迭代间不变），后续只调 `factorize() + solve()`。这在 Eigen 的 `SimplicialLDLT::compute()` 中是合在一起的，分离后可节省约 10-20% 的求解时间。

### File-by-File TODO

#### 1. `external/pbpt/src/pbpt/math/solver/linear_solver.hpp` — 新建

- [ ] `SolverStatus` 枚举
- [ ] `SolverBackend` concept
- [ ] `LinearSolver<T, Backend>` 模板类

#### 2. `external/pbpt/src/pbpt/math/solver/backend/suitesparse_ldlt.hpp` — 新建

- [ ] `SuiteSparseBackend` 实现
- [ ] CHOLMOD 初始化/清理
- [ ] CSR → cholmod_sparse 适配
- [ ] analyze / factorize / solve 三阶段

#### 3. `external/pbpt/src/pbpt/math/solver/backend/eigen_ldlt.hpp` — 新建（过渡）

- [ ] `EigenBackend` 实现
- [ ] CSR → Eigen::SparseMatrix 转换
- [ ] 包装 SimplicialLDLT

#### 4. `src/rtr/system/physics/ipc/solver/newton_solver.hpp` — 重写求解部分

- [ ] `Eigen::Triplet` → `pbpt::math::Triplet`
- [ ] `Eigen::SparseMatrix` → `pbpt::math::SparseMatrix`
- [ ] `SimplicialLDLT` → `LinearSolver<double, SuiteSparseBackend>`
- [ ] 分离 `analyze()` 到首次迭代
- [ ] 正则化重试逻辑适配新接口

#### 5. CMakeLists.txt — 修改

- [ ] 添加 SuiteSparse/CHOLMOD 的 `find_package` 和链接
- [ ] 确保条件编译：无 SuiteSparse 时 fallback 到 EigenBackend

### 验证方案

1. **单元测试**：
   - 3x3, 10x10 已知稀疏系统的精确解对比
   - 随机对称正定稀疏矩阵，SuiteSparse vs Eigen 后端结果 diff < 1e-12
   - `from_triplets` 重复项累加正确性

2. **集成测试**：
   - IPC 场景 100 帧，SuiteSparse 后端 vs 替换前 Eigen 版本逐帧对比 < 1e-10
   - 验证正则化重试机制正常工作

3. **性能基准**：
   - 对比 Eigen SimplicialLDLT vs SuiteSparse CHOLMOD 的求解时间
   - 预期 SuiteSparse 在 10K+ DOF 时快 2-5x

## 风险与缓解

| 风险 | 缓解策略 |
|------|---------|
| SuiteSparse 在 macOS 上编译配置复杂 | brew install suite-sparse，CMake FindSuiteSparse |
| CHOLMOD 对非正定矩阵行为不同于 SimplicialLDLT | 保留正则化重试逻辑，用 EigenBackend 交叉验证 |
| CSR/CSC 转换引入索引 bug | 对称矩阵只存下三角，CSR = CSC 无需转换 |
| 子矩阵提取改变非零结构，影响 analyze 复用 | free DOF 集合在时间步间不变时可复用，否则重新 analyze |
