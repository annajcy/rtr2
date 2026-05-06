# Phase -1: 清理 Physics 目录结构

在 Eigen 引入之前执行。目标是把物理目录收拢为两个干净的子系统：`rigid_body/`（含 collision）和 `ipc/`（新建）。

## 当前目录结构

```
src/rtr/system/physics/
├── physics_system.hpp
├── cloth/                    # 删除 — 被 IPC ShellBody 替代
├── collision/                # 移入 rigid_body/ — 只有 rigid_body 使用
├── common/                   # 删除 — ids/material 只服务 rigid_body，其余按需迁移
├── coupling/                 # 删除 — 空目录
├── fem/                      # 删除 — tet_surface_extract 迁入 ipc/
└── rigid_body/               # 保留
```

## 目标目录结构

```
src/rtr/system/physics/
├── physics_system.hpp        # 修改：去掉 ClothWorld
├── rigid_body/               # 保留 + collision 合并进来
│   ├── rigid_body.hpp
│   ├── rigid_body_type.hpp
│   ├── rigid_body_world.hpp
│   ├── collider.hpp
│   ├── contact.hpp
│   ├── collision/            # 从 physics/collision/ 移入
│   │   ├── collider_shape.hpp
│   │   ├── contact.hpp
│   │   ├── sphere_sphere.hpp
│   │   ├── sphere_plane.hpp
│   │   ├── sphere_box.hpp
│   │   ├── box_box.hpp
│   │   ├── box_plane.hpp
│   │   ├── mesh_plane.hpp
│   │   └── plane_common.hpp
│   ├── physics_ids.hpp       # 从 common/ 移入（只有 rigid 用）
│   ├── physics_material.hpp  # 从 common/ 移入
│   └── physics_step_context.hpp  # 从 common/ 移入
└── ipc/                      # 新建
    ├── core/
    ├── model/
    ├── energy/
    └── solver/
```

## 具体操作

### Step 1: collision/ -> rigid_body/collision/

collision 目录下的所有文件只被 `rigid_body_world.hpp` 和 `rigid_body/collider.hpp` 引用。

```bash
# 移动
mv src/rtr/system/physics/collision/ src/rtr/system/physics/rigid_body/collision/
```

修改 include 路径（2 个文件）：

- `rigid_body/rigid_body_world.hpp`:
  ```
  旧: #include "rtr/system/physics/collision/box_box.hpp"
  新: #include "rtr/system/physics/rigid_body/collision/box_box.hpp"
  ```
  共 7 个 collision include 需要改。

- `rigid_body/collider.hpp`:
  ```
  旧: #include "rtr/system/physics/collision/collider_shape.hpp"
  新: #include "rtr/system/physics/rigid_body/collision/collider_shape.hpp"
  ```

- `rigid_body/contact.hpp`:
  ```
  旧: #include "rtr/system/physics/collision/contact.hpp"
  新: #include "rtr/system/physics/rigid_body/collision/contact.hpp"
  ```

### Step 2: common/ 文件迁移

`common/` 下 5 个文件：

| 文件 | 处理 | 原因 |
|------|------|------|
| `physics_ids.hpp` | 移入 `rigid_body/` | RigidBodyID/ColliderID 只服务 rigid |
| `physics_material.hpp` | 移入 `rigid_body/` | 只有 rigid 用 |
| `physics_step_context.hpp` | 移入 `rigid_body/` | 只有 rigid 用 |
| `deformable_mesh_state.hpp` | 删除 | IPC 会用 Eigen IPCState 替代 |
| `normal_recompute.hpp` | 保留为顶层公共工具 | IPC 和 rigid 都可能用 |

`normal_recompute.hpp` 移到 `src/rtr/system/physics/normal_recompute.hpp`（与 `physics_system.hpp` 同级），或者直接留在 `common/` 如果你不想删空这个目录。推荐删 `common/` 并把 `normal_recompute.hpp` 提到 `physics/` 顶层。

需要修改引用 common 文件的 include 路径：

- `rigid_body_world.hpp` 中若引用 `common/physics_ids.hpp` -> `rigid_body/physics_ids.hpp`
- `cloth_scene_sync.hpp` 引用 `common/normal_recompute.hpp` -> 该文件本身也要删，见 Step 3

### Step 3: 删除 cloth/

涉及的文件和引用链：

**源文件（全删）：**
- `src/rtr/system/physics/cloth/` 整个目录（6 个文件）

**框架层（全删）：**
- `src/rtr/framework/component/physics/cloth/cloth_component.hpp`
- `src/rtr/framework/integration/physics/cloth_scene_sync.hpp`

**需要修改的引用方：**

1. `src/rtr/system/physics/physics_system.hpp`
   - 删除 `#include "rtr/system/physics/cloth/cloth_world.hpp"`
   - 删除 `ClothWorld m_cloth_world` 成员
   - 删除 `cloth_world()` 方法
   - 删除 `m_cloth_world.step(...)` 调用

2. `src/rtr/framework/integration/physics/scene_physics_step.hpp`
   - 删除 `#include "rtr/framework/integration/physics/cloth_scene_sync.hpp"`
   - 删除 `sync_scene_to_cloth(...)` 调用
   - 删除 `sync_cloth_to_scene(...)` 调用

3. `src/rtr/editor/panel/inspector_panel.hpp`
   - 删除 `#include "rtr/framework/component/physics/cloth/cloth_component.hpp"`
   - 删除 cloth 相关的 inspector 代码块

**测试（全删）：**
- `test/system/physics/cloth/` 整个目录（4 个文件）
- `test/framework/component/framework_cloth_component_test.cpp`
- `test/CMakeLists.txt` 中删除对应的 `rtr_add_test` 条目：
  - `test_cloth_mesh_topology_builder`
  - `test_cloth_state`
  - `test_cloth_spring`
  - `test_cloth_world`
  - `test_framework_cloth_component`
- `test/framework/integration/framework_scene_physics_step_test.cpp` 中删除 cloth 相关测试用例

**示例（全删或修改）：**
- `examples/editor/cloth_bunny_editor.cpp` — 删除或注释
- `examples/games103_lab/lab2_cloth/lab2_cloth.cpp` — 删除或注释
- `examples/editor/bunny_cloth_smoke_driver.hpp` — 删除或注释
- 检查 `examples/CMakeLists.txt` 删除对应 target

### Step 4: 删除 fem/

- `src/rtr/system/physics/fem/tet_surface_extract.hpp`
  - 这个逻辑后续会在 `ipc/model/tet_body.hpp` 中重新实现（作为 surface proxy 构建的一部分）
  - 或者直接移到 `ipc/` 下复用：`src/rtr/system/physics/ipc/model/tet_surface_extract.hpp`
  - **推荐先移到 ipc/model/ 下**，因为逻辑本身是正确的

- `test/system/physics/fem/tet_surface_extract_test.cpp`
  - 移到 `test/system/physics/ipc/tet_surface_extract_test.cpp`
  - 更新 include 路径
  - 更新 `test/CMakeLists.txt`

### Step 5: 删除 coupling/

空目录，直接删。

### Step 6: 删除 common/（如果全部迁出）

迁完后删除空目录。

对应测试：
- `test/system/physics/common/deformable_mesh_state_test.cpp` — 删除
- `test/system/physics/common/normal_recompute_test.cpp` — 移或删
- `test/CMakeLists.txt` 删除：
  - `test_deformable_mesh_state`
  - `test_normal_recompute`（如果 normal_recompute 也删了的话）

### Step 7: 新建 ipc/ 目录

```bash
mkdir -p src/rtr/system/physics/ipc/{core,model,energy,solver}
mkdir -p test/system/physics/ipc/solver
```

### Step 8: 更新 physics_system.hpp

清理后的 `physics_system.hpp`：

```cpp
#pragma once

#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    RigidBodyWorld m_rigid_body_world{};
    // IPCSystem m_ipc_system;  // Day 5 或更晚接入

public:
    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }

    void step(float delta_seconds) {
        m_rigid_body_world.step(delta_seconds);
    }
};

}  // namespace rtr::system::physics
```

## test/CMakeLists.txt 变更汇总

删除：
```cmake
test_cloth_mesh_topology_builder
test_cloth_state
test_cloth_spring
test_cloth_world
test_deformable_mesh_state
test_framework_cloth_component
```

修改 include 路径或移除：
```cmake
test_tet_surface_extract  # 路径从 fem/ 改到 ipc/
test_normal_recompute     # 路径更新或删除
test_collision_detection  # include 路径从 collision/ 改到 rigid_body/collision/
```

如有引用 cloth 的 framework integration test 也需清理。

## 执行顺序

```
1. collision -> rigid_body/collision (移动 + 改 include)
2. common 文件分流 (移动 + 改 include)
3. 删 cloth/ 全链路 (源/框架/编辑器/测试/示例)
4. fem/ -> ipc/model/ (移动 tet_surface_extract)
5. 删 coupling/
6. 删 common/ (如果已空)
7. 新建 ipc/ 目录结构
8. 更新 physics_system.hpp
9. 更新 test/CMakeLists.txt
10. cmake build + ctest 验证
```

## 验收

- `cmake --build --preset conan-debug` 成功
- `ctest --test-dir build/Debug -C Debug -LE integration` 全部通过（减去已删测试）
- `src/rtr/system/physics/` 下只有：`physics_system.hpp`、`rigid_body/`、`ipc/`、`normal_recompute.hpp`
- 无残留的 cloth/fem/common/coupling/collision 目录
- 无悬空 include
