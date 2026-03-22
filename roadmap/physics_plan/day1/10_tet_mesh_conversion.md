# Tet ↔ Mesh 互转工具

## 目标

在 IPC 的内部 tet 表示和引擎的 `ObjMeshData` 渲染表示之间实现**双向转换**：

- **Tet → Mesh**（Day 1）：把 IPC 求解后的变形位置写回可渲染的 `ObjMeshData`
- **Mesh → Tet**（后续）：从 OBJ surface mesh 经 fTetWild 生成完整四面体体网格

两个方向的复杂度完全不同。Tet→Mesh 是格式转换 + 法线重算；Mesh→Tet 是真正的 tetrahedralization，需要引入重量级第三方依赖。

## 为什么需要

| 方向 | 场景 | 复杂度 |
|------|------|--------|
| Tet → Mesh | 每帧把 IPC 求解后的变形位置写回渲染 mesh，供 Vulkan 管线显示 | 低：格式转换 + vertex remap + 法线 |
| Mesh → Tet | 从 OBJ 文件加载任意封闭 surface mesh，生成 IPC 可用的 tet body | 高：需要 volume meshing 算法 |

## 当前状态

- `tet_surface_extract.hpp`：只提取 surface triangle indices + surface vertex IDs，不生成完整的 `ObjMeshData`
- `ObjMeshData`（`rtr::utils::obj_types.hpp`）：渲染管线的标准 mesh 格式，含 position/uv/normal + triangle indices
- 两者之间没有桥梁
- **Mesh→Tet 方向完全缺失**——从 triangle surface mesh 到 tet volume mesh 不是简单格式转换，而是 tetrahedralization

## 数据结构对照

```text
IPC 侧 (Eigen)                           渲染侧 (pbpt::math)
─────────────────                         ────────────────────
Eigen::VectorXd x        (3N flat)   ↔   ObjVertex::position  (Vec3 per vertex)
std::array<size_t,4> tets              →  ObjMeshData::indices (uint32_t, triangles)
                                      ←   ObjMeshData::vertices (ObjVertex[])
Eigen::Vector3d                       ↔   pbpt::math::Vec3
```

---

# Part A: Tet → Mesh（Day 1 实现）

## 文件位置

`src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp`

替代现有 `tet_surface_extract.hpp`（后者的功能被包含在内）。

## 命名空间

`rtr::system::physics::ipc`

## 接口设计

```cpp
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "rtr/utils/obj_types.hpp"

namespace rtr::system::physics::ipc {

// ─── Tet → Mesh ────────────────────────────────────────────

struct TetSurfaceResult {
    std::vector<uint32_t> surface_indices;     // triangle indices (into original vertex array)
    std::vector<uint32_t> surface_vertex_ids;  // sorted unique vertex IDs on surface
};

/// 提取四面体网格的表面三角形（保留原有功能）
TetSurfaceResult extract_tet_surface(
    uint32_t vertex_count,
    std::span<const std::array<uint32_t, 4>> tets
);

/// 从 IPC 状态构建可渲染的 ObjMeshData
/// - positions: 从 IPCState::x 提取的当前顶点位置 (Eigen::VectorXd, size 3N)
/// - surface: 预计算好的 TetSurfaceResult
/// - 返回: 只包含 surface vertices 的 ObjMeshData，indices 已重映射
rtr::utils::ObjMeshData tet_to_mesh(
    const Eigen::VectorXd& positions,
    const TetSurfaceResult& surface
);

/// 轻量版：只更新已有 ObjMeshData 的 positions 和 normals
/// 用于每帧 sync，避免每帧重新分配内存
void update_mesh_positions(
    rtr::utils::ObjMeshData& mesh,
    const Eigen::VectorXd& positions,
    const TetSurfaceResult& surface
);

// ─── Mesh → Tet (input preparation helpers) ────────────────

/// 从 ObjMeshData 提取顶点位置为 Eigen 格式
/// 注意：这不是 tetrahedralization，只是格式转换
/// 用途：供外部 tet mesher (fTetWild) 使用
std::vector<Eigen::Vector3d> mesh_positions_to_eigen(
    const rtr::utils::ObjMeshData& mesh
);

/// 从 ObjMeshData 提取 surface triangles 为 index array
/// 用途：作为 tet mesher 的 surface constraint 输入
std::vector<std::array<uint32_t, 3>> mesh_triangles(
    const rtr::utils::ObjMeshData& mesh
);

}  // namespace rtr::system::physics::ipc
```

## 各函数实现要点

### `tet_to_mesh()`

1. 从 `surface.surface_vertex_ids` 建立 **old→new index 映射**（surface 上可能只有部分顶点）
2. 遍历 surface vertex IDs，从 `positions` 中提取 `Vec3`，构建 `ObjVertex`（uv 置零）
3. 用映射表把 `surface.surface_indices` 重新编号为紧凑的 0-based indices
4. 计算 per-face normal 并 accumulate 到顶点（和 `obj_io.hpp` 中 `load_obj_from_path` 的法线计算逻辑一致）

```text
vertex 映射示例:
  原始 tet mesh: vertex 0,1,2,...,26 (27 vertices, 3x3x3 block)
  surface vertex IDs: [0, 1, 2, 5, 7, 8, ...] (比如 20 个)
  new index map: {0→0, 1→1, 2→2, 5→3, 7→4, 8→5, ...}
  ObjMeshData.vertices: 20 个 ObjVertex
  ObjMeshData.indices: 使用 new indices
```

### `update_mesh_positions()`

- 不重新分配 vertices 和 indices
- 只更新 `mesh.vertices[i].position` 和重新计算法线
- 每帧调用一次，性能关键路径

```cpp
for (size_t i = 0; i < surface.surface_vertex_ids.size(); ++i) {
    uint32_t vid = surface.surface_vertex_ids[i];
    mesh.vertices[i].position = Vec3(
        float(positions[3*vid]),
        float(positions[3*vid+1]),
        float(positions[3*vid+2])
    );
}
recompute_normals(mesh);  // 同 obj_io 中的 face-area-weighted 法线
```

### `mesh_positions_to_eigen()` 和 `mesh_triangles()`

这两个函数**不做 tetrahedralization**，只是格式转换：

- `mesh_positions_to_eigen()`：遍历 `ObjMeshData::vertices`，逐个 `float→double` 转为 `Eigen::Vector3d`
- `mesh_triangles()`：把 flat `uint32_t` indices 重组为 `std::array<uint32_t, 3>`，前置检查 `indices.size() % 3 == 0`

它们的真实语义是 **`ObjMeshData → meshing input data`**，不是 **`ObjMeshData → TetGeometry`**。真正的 Mesh→Tet 见 Part B。

## 类型转换注意事项

| 转换 | 方向 | 注意 |
|------|------|------|
| `Eigen::Vector3d` → `pbpt::math::Vec3` | tet→mesh | `double→float` 精度损失，渲染可接受 |
| `pbpt::math::Vec3` → `Eigen::Vector3d` | mesh→tet | `float→double` 无损 |
| `std::size_t` → `uint32_t` | tet→mesh | tet connectivity 用 `size_t`，OBJ 用 `uint32_t`，需 cast |
| vertex reindexing | tet→mesh | surface 只用部分顶点，必须重映射 |

## 与现有代码的关系

### 替代 `tet_surface_extract.hpp`

`tet_mesh_convert.hpp` 完全包含 `extract_tet_surface()` 的功能。迁移后删除旧文件。

### 与 `normal_recompute.hpp` 的关系

`normal_recompute.hpp`（`physics/` 顶层）操作 `pbpt::math` 类型的法线重计算。`tet_to_mesh` 内部的法线计算是独立的（直接操作 `ObjMeshData`），不依赖它。如果未来要统一，可以把法线逻辑抽成共享 helper。

### 与 `obj_io.hpp` 的关系

`obj_io.hpp` 负责文件 I/O（load/write OBJ）。本模块负责内存中的数据转换。两者组合使用：

```text
加载 OBJ → obj_io::load_obj_from_path() → ObjMeshData
                                              ↓
                                    mesh_positions_to_eigen()
                                    mesh_triangles()
                                              ↓
                                    fTetWild tetrahedralization (Part B)
                                              ↓
                                         TetGeometry
                                              ↓
                                      IPC solver 使用
                                              ↓
                                    extract_tet_surface() (一次)
                                              ↓
                                       TetSurfaceResult
                                              ↓
                          tet_to_mesh() / update_mesh_positions() (每帧)
                                              ↓
                                         ObjMeshData
                                              ↓
                               渲染管线 / obj_io::write_obj_to_path()
```

## Part A 测试

替代现有 `tet_surface_extract_test.cpp`，新文件 `tet_mesh_convert_test.cpp`：

```text
TEST(TetMeshConvert, ExtractSurface_SingleTet)
  - 保留现有 extract_tet_surface 测试逻辑
  - 4 faces * 3 = 12 surface indices, 4 surface vertices

TEST(TetMeshConvert, ExtractSurface_SharedFace)
  - 保留现有共享面测试

TEST(TetMeshConvert, TetToMesh_SingleTet)
  - 构造 1 个 tet，positions 为 Eigen::VectorXd(12)
  - tet_to_mesh() 返回 ObjMeshData
  - vertices.size() == 4, indices.size() == 12
  - 每个 vertex.position 与输入一致（double→float 误差 < 1e-6）
  - normals 非零

TEST(TetMeshConvert, TetToMesh_VertexRemapping)
  - 构造 2 个 tet 共享一面（5 vertices）
  - 验证 ObjMeshData indices 是紧凑的 0-based
  - vertex count == surface_vertex_ids.size()

TEST(TetMeshConvert, UpdateMeshPositions)
  - tet_to_mesh() 得到初始 mesh → 修改 positions → update_mesh_positions()
  - 验证 position 已更新、normals 已重算

TEST(TetMeshConvert, MeshToEigen_RoundTrip)
  - ObjMeshData → mesh_positions_to_eigen() → 验证 float→double 正确

TEST(TetMeshConvert, MeshTriangles)
  - ObjMeshData with 6 indices → mesh_triangles() → 2 个 array<uint32_t,3>
```

## Part A 实现顺序

```text
1. 新建 tet_mesh_convert.hpp，把 extract_tet_surface 搬入        (~10 min)
2. 实现 tet_to_mesh() + 法线计算                                  (~20 min)
3. 实现 update_mesh_positions()                                    (~10 min)
4. 实现 mesh_positions_to_eigen() + mesh_triangles()               (~10 min)
5. 写测试 + 验证                                                   (~20 min)
6. 删除旧 tet_surface_extract.hpp，更新 CMakeLists.txt            (~5 min)
7. cmake build + ctest                                             (~5 min)
```

---

# Part B: Mesh → Tet — fTetWild 完整 Tetrahedralization

## 为什么 surface mesh 不能直接当 tet mesh 用

`ObjMeshData` 只有**表面三角形**——它描述的是物体的外壳，不知道物体内部应该怎样被剖分。而 IPC 的 FEM 求解需要**体网格**（四面体）：

- 弹性能 $\Psi(F)$ 是在每个四面体内部定义的
- 质量 $\rho V_e$ 是体积量，需要知道内部剖分
- deformation gradient $F = D_s D_m^{-1}$ 需要四面体的 4 个顶点

从 surface → volume 需要决定：内部采样点在哪、怎么连接成四面体、边界面如何保持、单元质量如何控制。这是 computational geometry 的核心问题，不是简单的索引重组。

## 后端选择：fTetWild

选择 fTetWild 而非 TetGen / CGAL 的原因：

| 工具 | 优点 | 缺点 |
|------|------|------|
| TetGen | 快、轻量、GPL | 对输入质量要求极高，稍有 self-intersection 就崩 |
| CGAL | 功能全、工业级 | 依赖链庞大，GPL/商业双许可，构建复杂 |
| **fTetWild** | **鲁棒性极强**，容忍 non-manifold/self-intersecting 输入，MPL-2.0 | 依赖较多（geogram），构建需要 patch |

fTetWild 的核心优势是鲁棒性：真实世界的 OBJ mesh 经常有 T-junction、self-intersection、非流形边等问题，fTetWild 都能处理。

## 架构决策

### 已编译 adapter target（不污染 header-only 架构）

fTetWild 和 geogram 的头文件不能暴露到 `rtr_runtime`（header-only INTERFACE library）。解决方案：

```text
rtr_ipc_meshing (STATIC library, 已编译)
  ├── 链接 ftetwild::ftetwild (PRIVATE)
  ├── public header: tet_meshing.hpp (只暴露项目类型)
  └── impl: tet_meshing.cpp (fTetWild 调用细节)

rtr_runtime (INTERFACE library)
  └── 依赖 rtr_ipc_meshing (INTERFACE)
```

这样 fTetWild/GEO 的头文件只出现在 `tet_meshing.cpp` 中，不泄漏到使用方。

### CMake option 控制可选编译

通过 `RTR_ENABLE_FTETWILD` 选项把 meshing adapter 做成可选 target：

```cmake
option(RTR_ENABLE_FTETWILD "Enable fTetWild-based tetrahedralization" OFF)

if(RTR_ENABLE_FTETWILD)
    find_package(ftetwild CONFIG REQUIRED)
    add_library(rtr_ipc_meshing STATIC
        src/rtr/system/physics/ipc/model/tet_meshing.cpp
    )
    target_link_libraries(rtr_ipc_meshing
        PUBLIC Eigen3::Eigen
        PRIVATE ftetwild::ftetwild
    )
    target_compile_definitions(rtr_runtime INTERFACE RTR_HAS_FTETWILD=1)
    target_link_libraries(rtr_runtime INTERFACE rtr_ipc_meshing)
endif()
```

好处：
- 没有 fTetWild 时，整个 IPC 模块仍然能编译（用 `generate_tet_block()` 或预生成的 tet mesh）
- `mesh_to_tet_*` 函数在 `#ifndef RTR_HAS_FTETWILD` 时编译为 `throw std::runtime_error("fTetWild not available")`
- CI 可以分 with/without fTetWild 两个配置跑

## 第三方依赖接入：自定义 Conan Recipe

### 为什么需要自定义 recipe

- fTetWild 没有 ConanCenter 包
- geogram 也没有 ConanCenter 包
- fTetWild upstream CMake 深度依赖 FetchContent

### Recipe 结构

```text
third_party/conan/recipes/ftetwild/
├── conanfile.py
├── patches/
│   ├── 001-disable-fetchcontent.patch
│   └── 002-geogram-vendored.patch
└── test_package/
    ├── conanfile.py
    └── test_ftetwild.cpp
```

### Recipe 设计要点

- `source()` 拉取固定 commit/tag 的 upstream `wildmeshing/fTetWild`
- 同时拉取固定版本的 geogram 源码，放入 recipe source 的 vendored 子目录
- Conan 依赖使用现成包：`eigen`, `spdlog`, `onetbb`, `gmp`, `libigl`, `cli11`, `fmt`
- Patch upstream CMake：
  - 禁掉 FetchContent 的 `fmt/spdlog/libigl/cli11/tbb`
  - geogram 改为 vendored 子目录
  - 只构建库目标 `FloatTetwild`，不构建 CLI 可执行文件
- 导出 target：`ftetwild::ftetwild`

### 工程风险：这是整个 plan 里最大的风险点

fTetWild upstream CMake 深度依赖 FetchContent，patch 掉所有 FetchContent 并改接 Conan targets 会很费力。geogram 有自己的 Vorpaline 构建系统，macOS 上尤其多平台特殊处理。

**建议**：先单独把 Conan recipe 跑通（`conan create` 成功），再做 adapter 集成。不要混在一起调试。

**构建时间**：fTetWild + geogram 编译很重。recipe 产出的预编译包应缓存到 Conan local cache，避免每次 clean build 重编。CI 上考虑用 Conan remote cache。

### 主工程构建改动

- `conanfile.py`：`self.requires("ftetwild/<version>-rtr")` （仅当 `RTR_ENABLE_FTETWILD`）
- `cmake/third_party.cmake`：`find_package(ftetwild CONFIG REQUIRED)`
- `src/CMakeLists.txt`：新增 `rtr_ipc_meshing` target

## 公开 API

新增 `src/rtr/system/physics/ipc/model/tet_meshing.hpp`：

```cpp
#pragma once

#include <Eigen/Core>
#include <array>
#include <cstddef>
#include <vector>

#include "rtr/utils/obj_types.hpp"

namespace rtr::system::physics::ipc {

struct TetGeometry {
    std::vector<Eigen::Vector3d> rest_positions;
    std::vector<std::array<std::size_t, 4>> tets;
    std::vector<Eigen::Matrix3d> Dm_inv;
    std::vector<double> rest_volumes;

    /// 计算 Dm_inv 和 rest_volumes，检查退化 tet
    void precompute_rest_data();
};

struct TetMeshingOptions {
    double ideal_edge_length_rel{0.05};  // 相对于 bbox 对角线
    double ideal_edge_length_abs{0.0};   // > 0 时优先于 rel
    double epsilon_rel{1e-3};            // 包络精度
    double stop_energy{10.0};            // 优化停止阈值
    int max_iterations{80};
    bool skip_simplify{false};
    bool coarsen{false};
    unsigned int max_threads{0};         // 0 = backend default
};

/// 完整 tetrahedralization：ObjMeshData → TetGeometry
/// 输入要求：closed, triangulated, orientable manifold
/// 内部流程：weld → validate → orient → fTetWild → normalize → precompute
TetGeometry mesh_to_tet_geometry(
    const rtr::utils::ObjMeshData& mesh,
    const TetMeshingOptions& options = {}
);

// 前向声明，TetBody 定义在 tet_body.hpp
struct TetBody;

/// 便捷入口：mesh → tet geometry → TetBody（含 precompute）
TetBody mesh_to_tet_body(
    const rtr::utils::ObjMeshData& mesh,
    const TetMeshingOptions& options = {}
);

}  // namespace rtr::system::physics::ipc
```

## `mesh_to_tet_geometry()` 完整数据流

### Step 1: Surface 准备 — exact-position weld

`ObjMeshData` 因 UV seam / hard normal 会复制空间位置相同的顶点。必须先焊接：

- 按完全相同的 `(x,y,z)` 浮点值合并（exact weld，不做 epsilon weld）
- 产出内部 `PreparedSurfaceMesh { vector<Vector3d> positions, vector<array<uint32_t,3>> triangles }`

### Step 2: 输入校验

- vertices 非空
- indices.size() % 3 == 0
- 每个 index 合法
- 每个三角形 3 个顶点互不相同
- 焊接后无零面积三角形
- **每条无向边恰好属于 2 个三角形**，否则判定为 open 或 non-manifold，抛 `std::invalid_argument`

### Step 3: 三角面方向统一

- 基于 triangle adjacency 做 BFS，保证共享边在相邻三角形中方向相反
- 对每个连通分量做一致定向
- 用 signed volume 判断整体朝向（内/外）

**嵌套壳的特殊情况**：如果有嵌套壳（如空心球），内层壳法线应该朝里（指向外壳和内壳之间的空间）。不能简单地"signed volume 为负就翻转"——需要判断该连通分量是外壳还是内壳。v1 可以只支持单连通分量，多壳情况报错或 warn。

### Step 4: 调用 fTetWild

- `PreparedSurfaceMesh` → `GEO::Mesh`
- `TetMeshingOptions` → `floatTetWild::Parameters` 映射：
  - `ideal_edge_length_abs > 0` 时写 abs，否则写 rel
  - `eps_rel = epsilon_rel`
  - `stop_energy = stop_energy`
  - `max_its = max_iterations`
  - `coarsen = coarsen`
  - `num_threads = max_threads == 0 ? max : max_threads`
  - `skip_simplify` 通过 `tetrahedralization(...)` 参数传入
- 固定内部参数：
  - `correct_surface_orientation = true`
  - `use_input_for_wn = true`
  - `smooth_open_boundary = false`
  - `manifold_surface = false`

关于 `manifold_surface`：设为 false 意味着输出表面不保证 manifold。如果后续 Day 3 要做 surface contact / barrier，可能需要改为 true，或在输出后额外做一次 surface extraction。这个选项后续根据 contact 需求调整。

### Step 5: 输出规范化

- `VO` → `vector<Vector3d> rest_positions`
- `TO` → `vector<array<size_t, 4>> tets`
- 对每个 tet 计算 `det(Dm)`，若为负则交换两个局部顶点，统一为正体积取向
- 空 tet 集合或退化 tet → `std::runtime_error`
- 调用 `precompute_rest_data()`

### 异常策略

| 阶段 | 异常类型 | 触发条件 |
|------|----------|----------|
| 输入校验 | `std::invalid_argument` | open mesh / non-manifold / degenerate triangle |
| 输入校验 | `std::out_of_range` | index 越界 |
| fTetWild 调用 | `std::runtime_error` | 返回失败码、空输出、非法 tet |
| 输出规范化 | `std::runtime_error` | 全部 tet 退化 |

## Part B 测试

新增 `test/system/physics/ipc/tet_meshing_test.cpp`：

```text
TEST(TetMeshing, CubeSurface)
  - 手写封闭立方体 ObjMeshData（8 vertices, 12 triangles）
  - mesh_to_tet_geometry() 返回非空 rest_positions 和 tets
  - 所有 tet index 合法
  - precompute_rest_data() 不抛异常

TEST(TetMeshing, CubeToBody)
  - mesh_to_tet_body() 返回 body
  - info.type == IPCBodyType::Tet
  - body.precompute() 通过

TEST(TetMeshing, WeldsDuplicatedRenderVertices)
  - 构造带 UV seam duplicate 的 ObjMeshData
  - weld 后应成功 tetrahedralize（不因 duplication 产生 open-edge 误判）

TEST(TetMeshing, RejectsOpenSurface)
  - 单片三角壳（open mesh）→ std::invalid_argument

TEST(TetMeshing, RejectsNonManifoldEdge)
  - 3 个三角形共享一条边 → std::invalid_argument

TEST(TetMeshing, RejectsDegenerateTriangle)
  - 共线顶点 → std::invalid_argument

TEST(TetMeshing, RejectsOutOfRangeIndices)
  - index 越界 → std::out_of_range

TEST(TetMeshing, OutputTetsHavePositiveOrientation)
  - 转换后逐个 tet 验证 det(Dm) > 0

TEST(TetMeshing, OptionMappingSmoke)
  - 传非默认参数，验证至少能跑通并产出非空 tet mesh
```

## 优先级与排期

```text
Day 1:  Part A (tet_mesh_convert.hpp)           ← 必须
        用 generate_tet_block() 生成 tet mesh
        不需要 Mesh→Tet

Day 3+: Part B (tet_meshing.hpp + fTetWild)     ← 后续
        建议分两步：
        Step 1: 单独跑通 Conan recipe (fTetWild + geogram)
        Step 2: 写 adapter + 测试
```

Part B 不应该和 Day 1 的 IPC solver 开发混在一起。Day 1 的 tet mesh 来源是 `generate_tet_block()`，完全够用。

## Assumptions

- fTetWild 的 MPL-2.0 许可在本项目内可接受
- 输入 mesh 只支持 triangulated、watertight、orientable manifold；坏输入直接拒绝，不做自动修复（no hole filling, no self-intersection repair）
- `ObjMeshData` 因 UV/normal 造成的重复顶点用 exact-position weld 处理
- `mesh_to_tet_body()` 使用 `TetBody` 默认密度和材料参数
- v1 只支持单连通分量的封闭表面，多壳/嵌套壳后续扩展
- 没有 fTetWild 时（`RTR_ENABLE_FTETWILD=OFF`），IPC 模块其余功能不受影响
