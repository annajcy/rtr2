# Phase 1: 核心数据结构

## 理论基础

### FEM 离散：从连续位移场到节点自由度

**参考：Lec 10 §2-3**（`lec10_finite_element/lec10_course_note.md`）

物理系统的状态是连续位移场 $x(X,t)$，有无穷多自由度。FEM 的核心思想是：**用有限个节点的坐标来代表整个连续场**。

推导链路：

1. **空间离散**：把域 $\Omega^0$ 剖分成不重叠的四面体单元，共享节点
2. **定义形函数 $N_a(X)$**：每个节点 $a$ 对应一个 shape function，满足插值性（$N_a(X_a) = 1$，$N_a(X_b) = 0$）和 partition of unity（$\sum_a N_a(X) = 1$）。线性四面体中 $N_a$ 就是重心坐标
3. **用形函数近似连续场**：

$$
\hat{x}(X) = \sum_a x_a N_a(X)
$$

其中 $x_a \in \mathbb{R}^3$ 是节点 $a$ 的当前坐标（未知量），$N_a(X)$ 是已知的形函数。

同样的离散化作用在测试函数 $Q(X)$ 上（Galerkin 方法），代入 weak form 后，"对任意连续函数 $Q$ 成立"变成"对每个节点 $a$ 各写一个方程"——总共 $n \times 3$ 个方程。

**结论**：全局状态向量 `IPCState::x` 就是所有节点坐标的堆叠 $\mathbf{x} = [x_1^T, \ldots, x_n^T]^T \in \mathbb{R}^{3n}$。后续所有操作——能量、梯度、Hessian——都只操作这个有限维向量。

关键推论：
- 因为 $N_a$ 是线性函数，$\nabla N_a$ 是常量 → $F = \partial\hat{x}/\partial X$ 在单元内是常量（Lec 10 §5.2）——极大简化积分
- 1 个 3D 节点 = 3 个 DOF，$n$ 个节点 → $3n$ DOF

---

## 目录结构

新建以下目录：

```
src/rtr/system/physics/ipc/
src/rtr/system/physics/ipc/core/
src/rtr/system/physics/ipc/model/
src/rtr/system/physics/ipc/energy/
src/rtr/system/physics/ipc/solver/
```

测试目录：

```
test/system/physics/ipc/
test/system/physics/ipc/solver/
```

## 命名空间

`rtr::system::physics::ipc`

与现有 `rtr::system::physics` 下的 cloth / rigid_body 保持一致的层级关系。

---

## File 1: `core/ipc_state.hpp`

统一全局 DOF 管理。所有 body 的顶点都映射到这个向量。

```cpp
namespace rtr::system::physics::ipc {

struct IPCState {
    Eigen::VectorXd x;          // 当前位形, 3N
    Eigen::VectorXd x_prev;     // 上一步位形
    Eigen::VectorXd v;          // 速度, 3N
    Eigen::VectorXd mass_diag;  // 质量对角, 3N (每个顶点 3 个相同值)

    void resize(std::size_t vertex_count);
    std::size_t vertex_count() const;
    std::size_t dof_count() const;  // = 3 * vertex_count

    // 顶点 i 的 position segment: x.segment<3>(3*i)
    Eigen::Ref<Eigen::Vector3d> position(std::size_t vertex_index);
    Eigen::Ref<const Eigen::Vector3d> position(std::size_t vertex_index) const;
};

}
```

要点：
- 1 顶点 = 3 DOF
- `mass_diag` 为对角质量，每个顶点的 3 个 DOF 共享同一质量值
- 不做稀疏质量矩阵，直接 VectorXd 对角存

验收：
- `resize(4)` 后 `dof_count() == 12`
- `position(2)` 返回 `x[6..8]` 的 segment

---

## File 2: `model/ipc_body.hpp`

最小公共 body 元数据。不用复杂继承，用 enum + struct。

```cpp
namespace rtr::system::physics::ipc {

enum class IPCBodyType { Tet, Shell, Obstacle };

struct IPCBodyInfo {
    IPCBodyType type{};
    std::size_t dof_offset{0};      // 在全局 DOF 向量中的起始位置
    std::size_t vertex_count{0};
    bool enabled{true};
};

}
```

要点：
- `dof_offset` 是字节偏移除以 sizeof(double) 后的 DOF index（即 `3 * vertex_offset`）
- Day 1 只有一个 body，offset = 0
- Shell / Obstacle 类型预留，不实现

---

## File 3: `model/tet_body.hpp`

Day 1 核心对象。

```cpp
namespace rtr::system::physics::ipc {

struct TetBody {
    IPCBodyInfo info{};

    // Rest geometry
    std::vector<Eigen::Vector3d> rest_positions;
    std::vector<std::array<std::size_t, 4>> tets;  // tet connectivity

    // Per-tet precomputed
    std::vector<Eigen::Matrix3d> Dm_inv;     // inv(Dm) for each tet
    std::vector<double> rest_volumes;         // |det(Dm)| / 6

    // Material — 物理参数
    double density{1000.0};    // kg/m^3
    double youngs_modulus{1e5};
    double poisson_ratio{0.3};

    // Material — 本构模型（per-body variant dispatch）
    // youngs_modulus/poisson_ratio 描述"多硬多弹"
    // material variant 描述"用什么应力-应变关系"
    TetMaterialVariant material{FixedCorotatedMaterial{}};

    // Boundary — per-vertex, per-axis Dirichlet constraint
    // 每个顶点 3 个 bool，分别控制 x/y/z 轴是否固定
    // {true,true,true} = stick DBC（完全固定）
    // {false,true,false} = y 轴固定，x/z 自由（沿 xz 平面 slip）
    // {false,false,false} = 完全自由
    struct AxisConstraint {
        std::array<bool, 3> fixed{false, false, false};

        bool is_stick() const { return fixed[0] && fixed[1] && fixed[2]; }
        bool is_free() const { return !fixed[0] && !fixed[1] && !fixed[2]; }
        bool any_fixed() const { return fixed[0] || fixed[1] || fixed[2]; }
    };
    std::vector<AxisConstraint> vertex_constraints;  // size = vertex_count

    // 便捷接口
    void fix_vertex(std::size_t v);                        // stick: 固定全部轴
    void fix_vertex_axis(std::size_t v, int axis);         // slip: 固定单个轴 (0=x,1=y,2=z)
    void fix_vertex_axes(std::size_t v, std::array<bool,3> axes); // 自由指定

    void precompute();  // 计算 Dm_inv, rest_volumes, 质量
};

}
```

### precompute() 的理论推导

precompute 要预计算三样东西：$D_m^{-1}$、rest volume、vertex mass。它们全部来自 FEM 离散的数学推导。

#### $D_m$ 和 $D_m^{-1}$：从形变梯度的定义反推

**出发点**：FEM 的核心量是 deformation gradient $F = \partial x / \partial X$，它描述参考构型到当前构型的局部线性映射。

对一个线性四面体单元 $e = (v_0, v_1, v_2, v_3)$，内部的位移是 affine（线性插值），所以 $F$ 在整个四面体内是常数。这意味着 $F$ 可以直接由顶点坐标算出，不需要积分。

**推导过程**：

变形把参考构型的边向量映射成当前构型的边向量：

$$
F(X_1 - X_0) = x_1 - x_0, \quad F(X_2 - X_0) = x_2 - x_0, \quad F(X_3 - X_0) = x_3 - x_0
$$

把三组关系并排写成矩阵形式：

$$
F \underbrace{[X_1-X_0,\; X_2-X_0,\; X_3-X_0]}_{D_m} = \underbrace{[x_1-x_0,\; x_2-x_0,\; x_3-x_0]}_{D_s}
$$

$$
F D_m = D_s \quad \Longrightarrow \quad \boxed{F = D_s \, D_m^{-1}}
$$

其中 $D_m$ 是参考构型（rest positions）的 3×3 边向量矩阵，$D_s$ 是当前构型的。

**为什么预计算 $D_m^{-1}$**：$D_m$ 完全由 rest positions 决定，在仿真过程中不变。而 $D_s$ 每步随顶点运动更新。所以 $D_m^{-1}$ 只算一次，之后每步只需要 $F = D_s \cdot D_m^{-1}$（一次矩阵乘法）。

**直觉验证**（Lec 7 §1.4）：

- 没有形变 → $D_s = D_m$ → $F = I$
- 纯旋转 → $D_s = R \, D_m$ → $F = R$
- 拉伸/剪切 → 全部编码在 $D_s D_m^{-1}$ 里

#### Rest volume：从行列式到体积

$$
V_e = \frac{|\det(D_m)|}{6}
$$

这个公式来自线性代数的几何解释：$D_m$ 的三列是从 $v_0$ 出发到 $v_1, v_2, v_3$ 的三条边，$|\det(D_m)|$ 是这三条边围成的平行六面体的体积，而四面体体积恰好是平行六面体的 $\frac{1}{6}$。

更进一步，$\det(D_m)$ 的**符号**反映四面体的朝向（左手/右手）。如果 $\det(D_m) \le 0$，意味着四个顶点共面（退化）或者顺序错误。这是 precompute 中必须检查的。

另外，rest volume 和 $J = \det(F)$ 有优美的联系：

$$
J = \det(F) = \frac{\det(D_s)}{\det(D_m)} = \frac{V_{\text{current}}}{V_{\text{rest}}}
$$

即 $J$ 就是"当前体积 / 参考体积"的局部度量。$J < 1$ 压缩，$J > 1$ 膨胀，$J < 0$ 翻转（element inversion）。

#### Mass lumping：从 consistent mass 到对角近似

**Consistent mass matrix**（Lec 10 §4）从 weak form 自然推出：

$$
M_{ab} = \int_{\Omega^0} \rho_0 N_a N_b \, dX
$$

$N_a, N_b$ 是形函数，$M_{ab}$ 衡量节点 $a$ 和 $b$ 通过材料密度的惯性耦合。这是一个稀疏但非对角的矩阵。

**Mass lumping** 把每行质量汇总到对角线上，等价于把每个四面体的总质量 $\rho V_e$ 均分给 4 个顶点：

$$
m_a = \sum_{e \ni a} \frac{\rho V_e}{4}
$$

这里 $e \ni a$ 表示"包含节点 $a$ 的所有四面体"。每个节点从它所属的所有四面体处各领 $\frac{1}{4}$ 的质量。

**为什么可以这样近似**（Lec 10 §4.3）：Lumping 丢掉了节点间惯性耦合（off-diagonal entries），但换来对角质量矩阵——$M^{-1}$ 就是逐元素取倒数，计算代价从稀疏矩阵求解变成 $O(n)$ 的向量运算。这在隐式 Euler 优化、line search 和 contact 处理中尤其方便。图形学仿真几乎都用 lumped mass。

### precompute() 实现步骤

对每个 tet `[v0, v1, v2, v3]`：

1. 计算 `Dm`：三条边向量组成的 3×3 矩阵
   ```
   Dm.col(0) = rest_positions[v1] - rest_positions[v0]
   Dm.col(1) = rest_positions[v2] - rest_positions[v0]
   Dm.col(2) = rest_positions[v3] - rest_positions[v0]
   ```
2. 计算 `Dm_inv[i] = Dm.inverse()`
3. 计算 `rest_volumes[i] = std::abs(Dm.determinant()) / 6.0`
4. 检查 `rest_volumes[i] > 0`，否则 tet 退化

### 质量计算

```
vertex_mass[v] += density * rest_volume / 4.0  (每个 tet 均分给 4 个顶点)
```

质量在 `IPCSystem` 组装时写入 `IPCState::mass_diag`。

验收：
- 单个正四面体 precompute 后 `Dm_inv` 和 `rest_volumes` 值正确
- 退化 tet (共面顶点) 抛异常

---

## File 4: `model/obstacle_body.hpp`

Day 1 只建占位。

```cpp
namespace rtr::system::physics::ipc {

struct ObstacleBody {
    IPCBodyInfo info{};
    // Day 1: placeholder
    // Day 3 will add: vertices, triangles, edges for contact
};

}
```

---

## Tet Block 生成工具

Day 1 需要一个程序化生成 tet mesh 的工具，不依赖外部文件。

建议放在测试 helper 或 `model/` 下：

```cpp
// 生成 nx*ny*nz 的规则 tet block
// 每个 cube 分成 6 个 tet
TetBody generate_tet_block(
    std::size_t nx, std::size_t ny, std::size_t nz,
    double spacing,
    Eigen::Vector3d origin
);
```

Day 1 推荐用 3x3x3 block，顶部一层顶点标记为 stick fixed（`fix_vertex(v)`）。

`vertex_constraints` 在 `generate_tet_block` 后默认全部为 free，由调用方按需设置。

验收：
- 生成的 mesh 所有 tet 体积 > 0
- 顶点数 = (nx+1)*(ny+1)*(nz+1)
- tet 数 = nx*ny*nz*6（每个 cube 6 tet）
