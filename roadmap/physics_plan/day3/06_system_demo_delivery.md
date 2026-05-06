# Phase 6: PhysicsSystem 接入 + Demo

## 目标

把 `IPCSystem` 接入主运行时循环，并交付一个可运行的 contact demo。

---

## PhysicsSystem 接入

当前 `IPCSystem` 通过 `ipc_scene_sync.hpp` 与 scene 交互，但还没有接入 `PhysicsSystem` 的主循环。

### 接入方式

最小侵入的方案：在 `PhysicsSystem::update()` 中调用 `IPCSystem::step()`。

具体做法取决于当前 `PhysicsSystem` 的结构。可能的接入方式：

1. **直接调用**：`PhysicsSystem` 持有 `IPCSystem` 指针，每帧调用 `step()`
2. **回调注册**：`PhysicsSystem` 提供 `register_subsystem()` 接口
3. **组件驱动**：通过 `IPCTetComponent` 的 tick 触发

选择哪种取决于现有架构。但 Day 3 的目标只是"能在主程序中跑起来"，不需要做完美的架构。

---

## Demo：tet block 落到地面

### 文件

```
examples/editor/ipc_ground_contact_editor.cpp
```

### 场景

- 3×3×3 tet block，初始 y = 1.5
- Ground plane obstacle，y = 0
- 无固定顶点（全部自由）
- 重力 g = -9.81

### 配置

```cpp
ipc::IPCConfig config{
    .dt = 1.0 / 60.0,
    .gravity = Eigen::Vector3d{0.0, -9.81, 0.0},
    .solver_params = {
        .max_iterations = 50,
        .gradient_tolerance = 1e-5,
    },
    .dhat_squared = 0.01,
    .barrier_stiffness = 1e4,
    .enable_contact = true,
};
```

### 初始化流程

```text
1. runtime + editor + scene + camera + light（照抄 Day 1 demo）
2. generate_tet_block(3, 3, 3, 0.3, origin)  // 不设 fixed_vertices
3. register tet body to ipc_system
4. generate_ground_plane(10.0, 0.0)
5. register obstacle body to ipc_system
6. ipc_system.initialize()
7. 创建渲染用的 DeformableMeshComponent + IPCTetComponent
8. （可选）创建 ground visual mesh
9. runtime.run()
```

### 预期行为

1. 前 1-2 秒：block 自由落体加速
2. 接近地面时：barrier 激活，block 减速
3. 最终：block 停在地面附近，略有弹跳后稳定
4. 长时间运行：不穿透、不 NaN

### 调参指南

| 问题 | 原因 | 调整 |
|------|------|------|
| 穿透地面 | κ 太小或 CCD 没生效 | 增大 barrier_stiffness；检查 CCD |
| 弹飞 | κ 太大 | 减小 barrier_stiffness |
| Newton 不收敛 | barrier Hessian 条件数差 | 增大 min_regularization；减小 κ |
| 悬浮空中 | dhat 太大 | 减小 dhat_squared |
| NaN | 距离函数退化 | 检查除零保护 |

---

## 参数面板（最小版本）

如果 editor runtime 支持 ImGui 面板，添加可调参数：

- `dt`
- `barrier_stiffness` (κ)
- `dhat_squared`
- `solver_params.max_iterations`
- `enable_contact` toggle
- Material params (Young's modulus, Poisson ratio)

如果不方便加面板，至少把关键参数提取为 demo 文件开头的常量，方便手动调。

---

## CMake 修改

```cmake
# examples/CMakeLists.txt
add_executable(ipc_ground_contact_editor editor/ipc_ground_contact_editor.cpp)
target_link_libraries(ipc_ground_contact_editor PRIVATE rtr::runtime rtr::editor)
```

## 输出

新增：
- `examples/editor/ipc_ground_contact_editor.cpp`

修改：
- `examples/CMakeLists.txt`
