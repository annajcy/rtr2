# IPC Fixed-End Block 示例

本文档说明最小 editor 示例：

`examples/editor/ipc_fixed_end_block_editor.cpp`

这个 example 不只是一个可视化 demo，它也是当前第一条完整打通的 runtime 路径：

```text
TetBody
  -> IPCSystem.step(dt)
  -> sync_ipc_to_scene()
  -> DeformableMeshComponent
  -> GPU mesh update
```

## 为什么选“固定一端”

当前 IPC runtime 已经有：

- 惯性能
- 重力能
- tet 弹性能
- Newton 求解
- 基于 `fixed_vertices` 的整顶点 Dirichlet 约束

但还没有：

- contact / barrier energy
- obstacle coupling
- 地面碰撞

所以如果继续做自由落体 block，它只会一直往下掉。  
“固定一端”的 cantilever 更适合作为当前最小示例，因为它：

- 一开始就能看到形变；
- 不会很快飞出视野；
- 不依赖还没接入的接触功能。

## 场景搭建

这个示例复用了标准 editor 样板：

- runtime + editor host
- forward editor pipeline
- 一个 camera
- 一个 point light
- 一个 ground quad 作为视觉参考

ground 只是视觉参考，不参与物理碰撞。

## Tet Body 构造

示例使用 `generate_tet_block(...)` 生成一个较细长的 block：

```cpp
auto body = ipc::generate_tet_block(6, 2, 2, 0.2, Eigen::Vector3d(-0.6, 1.4, -0.2));
```

相比一个近似立方体，这种细长形状在重力下更容易表现出 cantilever 的下垂。

## 固定端约束

当前 runtime 使用 per-vertex 的 `fixed_vertices`，所以这个示例固定的是整个端面，而不是单独一个点。

逻辑是：

1. 找到 `body.geometry.rest_positions` 中最小的 `x`
2. 对所有满足 `x == min_x`（加一个 epsilon）的顶点，标记为 fixed

这意味着该端面顶点集的 3 个平移自由度都会被约束。

为什么固定端面而不是单点：

- 更符合当前 DBC 表达
- 数值和视觉上都更稳定
- 更符合“悬臂梁固定端”的直觉

## 注册 helper 的作用

示例里用了一个很小的本地 helper 来收口初始化样板代码：

```text
TetBody
  -> TetSurfaceMapping
  -> initial ObjMeshData
  -> DeformableMesh resource
  -> DeformableMeshComponent
  -> IPCSystem registration
  -> IPCTetComponent
```

这个 helper 的目的只是让 example 更清楚，不是引入一层新的 engine-wide 抽象。

## 运行时行为

场景初始化之后，这个 example **不会**在 `on_post_update` 里手写 mesh write-back。

真正的 deformable 更新路径是：

```text
fixed tick
  -> step_scene_physics(...)
      -> ipc_system.step(dt)
      -> sync_ipc_to_scene(...)
```

这样 callback 层只剩 editor UI 相关职责，例如：

- `EditorHost::begin_frame(...)`
- 相机 aspect ratio 更新
- `ESC` 关闭窗口

这也是它和旧版“手动回写 demo”最大的架构差异。

## 时间步对齐

现在 `IPCSystem` 直接使用 `step(delta_seconds)` 传入的时间步，所以 example 里会显式设置：

```cpp
AppRuntimeConfig.fixed_delta_seconds = 0.01;
```

这样 runtime fixed tick 和 IPC 求解时间步保持一致。

## 正确运行时应该看到什么

如果示例运行正确，你会看到：

- 左端固定在空中；
- 自由端在重力下向下弯曲；
- 表面法线正确，光照不会突然发黑或发白；
- 每帧用户回调中不再需要手写 `tet_to_mesh(...)`。

## 当前限制

这个示例仍然反映了当前 runtime 的边界：

- 没有地面碰撞
- 没有 obstacle body
- 没有 contact stabilization
- 还没有专门的 IPC Inspector 控件

这对当前示例是可以接受的，因为它的目标是验证 runtime bridge 和 deformable write-back 路径，而不是完整物理交互。
