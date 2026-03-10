# 相机与坐标系约定

这份文档记录 PBPT 与 RTR2 共享的坐标系和相机矩阵变换约定。

它主要回答三个问题：

1. 哪个轴被认为是“前方”？
2. 哪些 API 使用正的裁剪距离，哪些 API 使用带符号的 camera-space 平面值？
3. 从 object space 到 shader，再到 editor picking，完整矩阵链到底是什么？

## 统一约定

当前引擎全链路采用以下约定：

- 右手系
- 场景节点（Scene Node）局部坐标轴：
  - `+X`：右
  - `+Y`：上
  - `+Z`：节点 front
  - `-Z`：节点 back
- 相机 / view space：
  - 位于相机前方、可见的点满足 `z_view < 0`
  - 因此相机 forward 是 `-Z`
- NDC 深度范围：
  - near plane 映射到 `0`
  - far plane 映射到 `1`

这意味着“节点 front”和“相机 front”不是同一个定义：

- 普通场景节点把 `+Z` 当作 front
- 相机组件把节点的 `-Z` 当作相机 front

## Node 轴与 Camera 轴

Scene node 的基向量定义在 [`src/rtr/framework/core/scene_graph.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/core/scene_graph.hpp)：

- `local_front()` / `world_front()` = 旋转后的 `(0, 0, +1)`
- `local_back()` / `world_back()` = 旋转后的 `(0, 0, -1)`

Camera 组件在 [`src/rtr/framework/component/camera/camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/camera.hpp) 里对这套 node 约定做了一层适配：

- `camera_world_front()` 返回 `node.world_back()`
- `camera_world_back()` 返回 `node.world_front()`
- `camera_look_at_*()` 会先把目标方向取负，再交给 node

这层适配是故意保留的。这样 scene graph 仍然可以维持简单的 `+Z = front` 规则，而渲染链继续遵守“相机前方点在 view space 中 `z < 0`”这一常见约定。

## View Matrix 约定

view matrix 始终定义为：

```cpp
view = inverse(camera_world_matrix)
```

实现见 [`src/rtr/framework/component/camera/camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/camera.hpp)。

PBPT 的 `look_at(...)` 也遵守同样的 `-Z forward` camera-space 约定，实现在 [`external/pbpt/src/pbpt/math/matrix/matrix_transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/math/matrix/matrix_transform.hpp)：

- 相机前方的点经过 view 变换后满足 `z_view < 0`
- 透视投影后满足 `clip.w = -z_view`

## Near / Far 的语义边界

这里有两层 API，它们使用的不是同一组语义。

### 高层相机 API 使用正距离

以下接口把 near / far 解释为“距离相机的正距离”：

- PBPT 相机投影辅助层 [`external/pbpt/src/pbpt/camera/camera.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/camera/camera.hpp)
- PBPT XML 里的 `near_clip` / `far_clip`
- RTR2 相机组件：
  - [`src/rtr/framework/component/camera/perspective_camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/perspective_camera.hpp)
  - [`src/rtr/framework/component/camera/orthographic_camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/orthographic_camera.hpp)

典型写法：

- `near = 0.1`
- `far = 100.0`

### 低层投影数学 API 使用带符号的 camera-space 平面值

以下接口要求传入真正的 camera-space 平面位置：

- [`external/pbpt/src/pbpt/math/matrix/matrix_transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/math/matrix/matrix_transform.hpp)
- [`external/pbpt/src/pbpt/geometry/transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/geometry/transform.hpp)

在当前 `-Z forward` 约定下，应该传：

- `near_z = -abs(near_distance)`
- `far_z = -abs(far_distance)`

典型值：

- `near_z = -0.1`
- `far_z = -100.0`

RTR2 的 camera 组件会在调用低层 PBPT 数学函数前完成这次转换。

## 投影矩阵规则

### Orthographic

`pbpt::math::orthographic(left, right, bottom, top, near_z, far_z)` 假设：

- `near_z < 0`
- `far_z < near_z`
- near 映射到 `ndc.z = 0`
- far 映射到 `ndc.z = 1`

### Perspective

`pbpt::math::perspective(fov_y, aspect, near_z, far_z)` 也假设输入的是 signed camera-space planes。

这里有两个细节必须同时满足：

1. `near_z` / `far_z` 作为带符号平面参与 `z/w` 映射。
2. frustum 的宽高必须使用 `abs(near_z)` 来计算，而不能直接用负的 `near_z`。

如果直接拿负的 `near_z` 去算 `left/right/top/bottom`，`x` 和 `y` 会同时翻转，视觉上就会变成整张图绕光轴转了 180 度，也就是“上下和左右都反了”。

## 从 Object 到屏幕的矩阵链

RTR2 使用的 CPU / GPU 变换链是：

```text
object -> world -> view -> clip -> ndc
```

更具体地说：

```text
world_pos = model * local_pos
view_pos  = view  * world_pos
clip_pos  = proj  * view_pos
ndc       = clip.xyz / clip.w
```

相关文件：

- 场景收集：[`src/rtr/framework/integration/render/forward_scene_view_builder.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/integration/render/forward_scene_view_builder.hpp)
- UBO 打包：[`src/rtr/system/render/pipeline/forward/forward_pipeline.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/system/render/pipeline/forward/forward_pipeline.hpp)
- Shader 乘法链：[`shaders/vert_buffer.slang`](https://github.com/annajcy/rtr2/blob/main/shaders/vert_buffer.slang)

## 矩阵存储与 Shader 合约

Shader 中的 UBO 定义为：

```slang
row_major float4x4 model;
row_major float4x4 view;
row_major float4x4 proj;
```

实际乘法顺序为：

```slang
mul(ubo.model, position)
mul(ubo.view, world_pos)
mul(ubo.proj, view_pos)
```

CPU 侧 [`pack_mat4_row_major(...)`](https://github.com/annajcy/rtr2/blob/main/src/rtr/system/render/pipeline/forward/forward_pipeline.hpp) 直接按行写入 UBO，不再使用任何 transpose 兼容层。

这里希望始终成立的约束是：

```text
GPU 里的 clip 结果 == CPU 上 proj * view * model * position 的结果
```

## Editor Picking 约定

Editor picking 在 [`src/rtr/editor/core/scene_picking.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/editor/core/scene_picking.hpp) 里和渲染路径共用同一套投影语义：

- clip/NDC near depth = `0`
- clip/NDC far depth = `1`
- `inv_view_proj = inverse(proj * view)`

pick ray 的构造方式是：

- near 点使用 `(ndc_x, ndc_y, 0, 1)`
- far 点使用 `(ndc_x, ndc_y, 1, 1)`

这必须和 render path 完全一致。如果渲染链用的是 `[0, 1]` 深度，而 picking 仍然按 `[-1, 1]` 做反投影，即使画面看起来正常，选取结果也会是错的。

## PBPT Serde 的边界约定

PBPT XML 和 RTR2 scene/component API 对外都暴露正的 clip distance。真正的符号转换只发生在构建低层投影矩阵时。

相关文件：

- PBPT XML camera serde：[`external/pbpt/src/pbpt/serde/domain/impl/camera.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/serde/domain/impl/camera.hpp)
- RTR2 PBPT import：[`src/rtr/framework/integration/pbpt/serde/load/mappers.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/integration/pbpt/serde/load/mappers.hpp)
- RTR2 PBPT export：[`src/rtr/framework/integration/pbpt/serde/scene_writer.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/integration/pbpt/serde/scene_writer.hpp)

规则是：

- 外部配置和组件字段保持正值
- 只有低层矩阵数学函数接收负的 signed plane

## 调试时的快速检查表

遇到 camera / projection / picking 问题时，优先检查这几条：

1. 相机前方的点是否满足 `z_view < 0`。
2. 可见点在透视投影后是否满足 `clip.w > 0`。
3. 对外 near/far 是否保持正值，只有低层数学层才使用 signed plane。
4. NDC depth 是否使用 `[0, 1]`，而不是 `[-1, 1]`。
5. CPU 乘法、UBO 打包和 shader 乘法顺序是否一致。
6. picking 的反投影是否使用 `z = 0/1`。

如果某一层破坏了这些假设，常见症状通常是：

- 黑屏：`clip.w` 或 depth convention 不匹配
- 只有上下颠倒：viewport / raster-space 的 `y` 约定不一致
- 上下和左右都反：用 signed negative near 直接计算了 frustum 宽高
- picking 选不中或方向反：render path 与 unprojection 的 depth 约定不一致
