# Phase 9 — fTetWild 集成：Mesh-to-Tet 转换

> Naming note after the converter refactor:
> current file path is `src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp`
> instead of the older `src/rtr/system/physics/ipc/model/mesh_to_tet.hpp`.

## 目标

支持从任意三角形表面网格（OBJ / glTF）自动生成体四面体网格（`TetGeometry`），用于 IPC 仿真。
使用 [fTetWild](https://github.com/wildmeshing/fTetWild) 作为后端四面体化引擎。

---

## 1. 添加 fTetWild 为 Git Submodule

```bash
cd /Users/jinceyang/Desktop/codebase/graphics/rtr2
git submodule add https://github.com/wildmeshing/fTetWild.git external/fTetWild
git submodule update --init --recursive
```

> fTetWild 自身依赖 `libigl`、`geogram`、`fmt` 等，均通过其 CMakeLists.txt 内部的 `FetchContent` / submodule 管理。

### 目录结构

```
external/
├── pbpt/           # 已有
└── fTetWild/       # 新增 submodule
    ├── CMakeLists.txt
    ├── src/
    └── ...
```

---

## 2. CMake 集成

### 2.1 根 CMakeLists.txt 修改

在 `add_subdirectory(external/pbpt)` 之后添加：

```cmake
# fTetWild — tetrahedral meshing
option(RTR_BUILD_FTETWILD "Build fTetWild for mesh-to-tet conversion" ON)
if(RTR_BUILD_FTETWILD)
  # fTetWild 默认会构建可执行文件，我们只需要库
  set(FLOAT_TETWILD_WITH_EXACT_ENVELOPE OFF CACHE BOOL "" FORCE)
  add_subdirectory(external/fTetWild)
  message(STATUS "fTetWild enabled for mesh-to-tet conversion.")
endif()
```

### 2.2 链接到 rtr_runtime

在 `src/CMakeLists.txt` 中为 `rtr_runtime` 添加：

```cmake
if(RTR_BUILD_FTETWILD)
  target_link_libraries(rtr_runtime PUBLIC FloatTetwild)
  target_compile_definitions(rtr_runtime PUBLIC RTR_HAS_FTETWILD=1)
endif()
```

### 2.3 注意事项

| 事项 | 说明 |
|------|------|
| **C++ 标准** | fTetWild 使用 C++11/14；项目使用 C++23，二者兼容 |
| **Eigen 版本** | fTetWild 自带 Eigen，可能与 Conan 提供的 Eigen 冲突。若冲突，需设置 `set(FLOAT_TETWILD_EXTERNAL_EIGEN ON)` 并指向项目 Eigen |
| **编译时间** | fTetWild + geogram 首次编译约 3-5 分钟 |
| **平台** | macOS (Darwin) 已验证可用 |
| **可选裁剪** | 如果只需要库不需要 CLI，可设置 `set(FLOAT_TETWILD_BUILD_TESTS OFF CACHE BOOL "" FORCE)` |

---

## 3. C++ Adapter 层

### 3.1 新文件

```
src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp
```

### 3.2 接口设计

```cpp
#pragma once

#include <vector>
#include <Eigen/Core>
#include "rtr/system/physics/ipc/model/tet_body.hpp"

namespace rtr::system::physics::ipc {

/// fTetWild 四面体化参数
struct TetMeshingParams {
    double ideal_edge_length = 0.05;   // 理想边长（相对于 bounding box 对角线）
    double epsilon = 1e-3;             // 包络精度
    int max_iterations = 80;           // 最大优化迭代
    bool skip_simplification = false;  // 跳过简化步骤（调试用）
};

/// 输入：三角形表面网格
struct TriangleMeshInput {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<Eigen::Vector3i> triangles;
};

/// 输出：体四面体网格
struct TetMeshOutput {
    std::vector<Eigen::Vector3d> vertices;
    std::vector<std::array<std::size_t, 4>> tets;
    bool success = false;
    std::string error_message;
};

/// 核心转换函数
TetMeshOutput tetrahedralize(const TriangleMeshInput& input,
                             const TetMeshingParams& params = {});

/// 便利函数：直接生成 TetGeometry
TetGeometry triangle_mesh_to_tet_geometry(const TriangleMeshInput& input,
                                           const TetMeshingParams& params = {});

/// 便利函数：直接生成 TetBody（含材质）
TetBody triangle_mesh_to_tet_body(const TriangleMeshInput& input,
                                   const TetMaterialVariant& material = FixedCorotatedMaterial{},
                                   const TetMeshingParams& params = {});

} // namespace rtr::system::physics::ipc
```

### 3.3 实现要点

```cpp
// mesh_to_tet.cpp (或保持 header-only 带 inline)

#ifdef RTR_HAS_FTETWILD
#include <floattetwild/FloatTetWild.h>
#endif

TetMeshOutput tetrahedralize(const TriangleMeshInput& input,
                              const TetMeshingParams& params) {
#ifdef RTR_HAS_FTETWILD
    // 1. 将 input 转换为 fTetWild 期望的 Eigen 矩阵格式
    //    Eigen::MatrixXd V (n×3), Eigen::MatrixXi F (m×3)
    // 2. 调用 floatTetWild::tetrahedralization(V, F, ...)
    // 3. 将结果转回 TetMeshOutput
    // 4. 验证：检查 tet 方向（行列式 > 0），必要时翻转
#else
    return TetMeshOutput{.success = false,
                         .error_message = "fTetWild not available (RTR_HAS_FTETWILD=0)"};
#endif
}
```

### 3.4 条件编译策略

- `RTR_HAS_FTETWILD` 编译宏控制 fTetWild 代码路径
- 不启用时函数返回失败结果，不会编译错误
- `generate_tet_block()` 保留不动，两种方式共存

---

## 4. 与现有数据流的对接

### 4.1 数据流

```
OBJ/glTF 文件
    ↓  (现有 loader)
TriangleMeshInput {vertices, triangles}
    ↓  (fTetWild)
TetMeshOutput {vertices, tets}
    ↓  (转换)
TetGeometry → TetBody → IPCTetComponent → IPCSystem
```

### 4.2 从现有 OBJ Loader 桥接

项目已有 `tinyobjloader` 依赖和 `rtr/utils/obj_types.hpp`。需要一个转换函数：

```cpp
TriangleMeshInput obj_mesh_to_triangle_input(const utils::ObjMeshData& mesh);
```

### 4.3 从 glTF Loader 桥接

项目已有 `TinyGLTF` 依赖。可类似添加：

```cpp
TriangleMeshInput gltf_mesh_to_triangle_input(/* glTF mesh data */);
```

---

## 5. 使用示例

```cpp
// 加载 OBJ
auto mesh = load_obj("bunny.obj");
auto tri_input = obj_mesh_to_triangle_input(mesh);

// 四面体化
TetMeshingParams params;
params.ideal_edge_length = 0.03;  // 更细网格

auto tet_body = triangle_mesh_to_tet_body(
    tri_input,
    FixedCorotatedMaterial{.mass_density = 1000.0,
                           .youngs_modulus = 1e5,
                           .poisson_ratio = 0.3},
    params
);

// 加入场景
tet_body.precompute();
auto& component = game_object.add_component<IPCTetComponent>(std::move(tet_body));
```

---

## 6. 实施顺序 Checklist

### Phase A — Submodule & Build（预计 30 min）

- [ ] `git submodule add` fTetWild 到 `external/fTetWild`
- [ ] 修改根 `CMakeLists.txt`，`add_subdirectory(external/fTetWild)`
- [ ] 修改 `src/CMakeLists.txt`，链接 `FloatTetwild`
- [ ] 解决 Eigen 版本冲突（如果有）
- [ ] 验证：项目整体能编译通过

### Phase B — Adapter API（预计 1 h）

- [ ] 新建 `src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp`
- [ ] 实现 `tetrahedralize()` 核心函数
- [ ] 实现 `triangle_mesh_to_tet_geometry()` 便利函数
- [ ] 实现 `triangle_mesh_to_tet_body()` 便利函数
- [ ] 实现 `obj_mesh_to_triangle_input()` 桥接函数

### Phase C — 测试 & 验证（预计 30 min）

- [ ] 单元测试：简单立方体表面 → tet → 验证 tet 数量 > 0、方向正确
- [ ] 集成测试：bunny.obj → tet → IPCTetComponent → 运行 IPC step
- [ ] 验证与 `generate_tet_block()` 生成的 TetBody 走相同的仿真路径

### Phase D — Demo（可选）

- [ ] 在 examples 中新增 `ipc_mesh_to_tet_example`
- [ ] 加载一个 OBJ 模型（如 bunny），四面体化后跑 IPC 仿真

---

## 7. 风险与备选方案

| 风险 | 影响 | 应对方案 |
|------|------|----------|
| fTetWild CMake 与项目冲突 | 编译失败 | 用 `FetchContent` 替代 submodule，或隔离 build |
| Eigen 版本冲突 | 链接/运行时错误 | 设置 `FLOAT_TETWILD_EXTERNAL_EIGEN` 使用项目 Eigen |
| fTetWild 对某些网格失败 | 无法四面体化 | 在 `TetMeshOutput` 中返回错误；回退到 `generate_tet_block()` |
| 编译时间过长 | 开发体验差 | `RTR_BUILD_FTETWILD=OFF` 关闭；CI 缓存 |
| fTetWild API 变更 | 适配层失效 | 锁定 submodule 到稳定 commit |

---

## 8. 新增 / 修改文件清单

| 文件 | 操作 | 说明 |
|------|------|------|
| `external/fTetWild/` | 新增 | git submodule |
| `.gitmodules` | 修改 | 新增 fTetWild 条目 |
| `CMakeLists.txt` | 修改 | `add_subdirectory(external/fTetWild)` |
| `src/CMakeLists.txt` | 修改 | 链接 `FloatTetwild`，定义 `RTR_HAS_FTETWILD` |
| `src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` | 新增 | 核心 adapter API |
| `test/physics/ipc/mesh_to_tet_test.cpp` | 新增 | 单元测试 |
| `examples/ipc_mesh_to_tet_example.cpp` | 新增（可选） | Demo |
