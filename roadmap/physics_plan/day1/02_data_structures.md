# Phase 1: 核心数据结构

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

    // Material
    double density{1000.0};    // kg/m^3
    double youngs_modulus{1e5};
    double poisson_ratio{0.3};

    // Boundary
    std::vector<bool> fixed_vertices;  // true = Dirichlet

    void precompute();  // 计算 Dm_inv, rest_volumes, 质量
};

}
```

### precompute() 做的事

对每个 tet `[v0, v1, v2, v3]`：

1. 计算 `Dm`：三条边向量组成的 3x3 矩阵
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

Day 1 推荐用 3x3x3 block，顶部一层顶点标记为 fixed。

验收：
- 生成的 mesh 所有 tet 体积 > 0
- 顶点数 = (nx+1)*(ny+1)*(nz+1)
- tet 数 = nx*ny*nz*6（每个 cube 6 tet）
