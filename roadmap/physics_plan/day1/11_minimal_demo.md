# 最小示例：IPC Fixed-End Block

> Naming note after the converter refactor:
> `TetSurfaceResult` -> `TetSurfaceMapping`
> `extract_tet_surface()` -> `build_tet_surface_mapping()`
> `tet_to_mesh()` -> `tet_rest_to_surface_mesh()` / `tet_dofs_to_surface_mesh()`
> `update_mesh_positions()` -> `update_surface_mesh_from_tet_dofs()`

## 目标

先把 IPC 的 scene 桥接生命周期补齐，再写一个最小 editor example 验证整条链路：

```text
Scene + IPCTetComponent
    -> IPCSystem.step()
    -> sync_ipc_to_scene()
    -> DeformableMeshComponent
    -> GPU 渲染
```

这次最小示例不再走“demo 自己在 `on_post_update` 手动 writeback”的路线，而是先实现和 rigid body 类似的 IPC scene sync 生命周期。

## 这次方案相对旧版的调整

旧版 `11_minimal_demo.md` 的思路是：

- 不实现 ECS bridge
- 不改 `scene_physics_step.hpp`
- demo 自己在 `on_post_update` 里手动 `tet_to_mesh` / `apply_deformed_surface`
- 示例内容是一个自由下落的 tet block

现在改成：

- 先实现 `IPCTetComponent`
- 先实现 `ipc_scene_sync.hpp`
- 在 `scene_physics_step.hpp` 里接入 IPC 的 scene 生命周期
- 示例改成“固定一端”的版本，而不是自由落体

原因很简单：

1. 这条 scene bridge 迟早要补，而且它才是 IPC 接入 runtime/framework 的正确位置
2. 固定一端的 block 不依赖接触和碰撞，更适合作为当前阶段的稳定最小示例
3. 当前 `TetBody` 的 DBC 还是 `fixed_vertices`，天然更适合做“固定一端”而不是“底面 slip”之类更细粒度约束

---

## 当前范围

### 这次要做

1. `IPCTetComponent`
2. `ipc_scene_sync.hpp`
3. `scene_physics_step.hpp` 中的 IPC sync 生命周期接入
4. 一个固定一端的 editor example

### 这次不做

- contact / barrier / friction
- `mesh -> tet` tetrahedralization
- Inspector 专门的 IPC 面板
- 通用 kinematic scene -> IPC 每帧回写
- 更细粒度的 per-axis DBC

---

## 为什么示例改成“固定一端”

当前系统已经有：

- 重力
- 惯性能
- tet 材料能量
- Newton solver
- `fixed_vertices` 顶点级 Dirichlet

但还没有：

- 地面碰撞
- 接触约束
- barrier energy

所以如果继续做“自由落体 block”，视觉上它只会一直掉下去，无法形成一个稳定、可持续观察的最小示例。

“固定一端”的版本更适合当前实现：

- 一端固定，另一端在重力下下垂
- 不依赖任何 contact
- 能直接看到弹性形变
- 运行过程中不会飞离视野太快

这里的“固定一端”在实现上采用**固定一端面上的所有顶点**，而不是只固定一个单独顶点。  
原因是当前 DBC 粒度是 `fixed_vertices`，并且固定一个端面比只固定一个顶点稳定得多，也更符合“cantilever”最小演示的直觉。

---

## 场景生命周期设计

目标是让 IPC 的 scene 生命周期尽量对齐 rigid body 这套现有结构。

### rigid body 现状

当前 rigid body 的生命周期是：

```text
sync_scene_to_rigid_body(scene, rigid_body_world)
    -> physics_system.step(delta_seconds)
    -> sync_rigid_body_to_scene(scene, rigid_body_world)
```

### IPC 目标形态

IPC 这次接成：

```text
scene setup / registration
    -> IPCSystem.initialize()

每个 fixed tick:
    sync_scene_to_rigid_body(...)
    -> physics_system.step(delta_seconds)
    -> sync_rigid_body_to_scene(...)
    -> ipc_system.step()
    -> sync_ipc_to_scene(scene, ipc_system)
```

注意：

- Day 1 范围内，IPC **不需要**像 rigid body 那样每帧做 scene -> IPC 的动态同步
- 因为当前 `TetBody` 是初始化后静态注册的，节点拓扑和参考构型不会每帧被 scene 改写
- 所以这次的“ipc <-> scene sync 生命周期”里，真正每帧发生的是 **IPC -> scene writeback**
- scene -> IPC 在当前阶段只体现在**初始化注册**这一侧

---

## 新增/修改文件

| 类型 | 文件 | 作用 |
|------|------|------|
| 新建 | `src/rtr/framework/component/physics/ipc/ipc_tet_component.hpp` | scene 中的 IPC tet 桥接组件 |
| 新建 | `src/rtr/framework/integration/physics/ipc_scene_sync.hpp` | `sync_ipc_to_scene()` |
| 修改 | `src/rtr/framework/integration/physics/scene_physics_step.hpp` | 在 IPC step 后调用 `sync_ipc_to_scene()` |
| 新建 | `examples/editor/ipc_fixed_end_block_editor.cpp` | 固定一端的最小示例 |
| 修改 | `examples/CMakeLists.txt` | 注册新 example target |

如果要补测试，建议同时加：

| 类型 | 文件 | 作用 |
|------|------|------|
| 新建 | `test/framework/integration/physics/ipc_scene_sync_test.cpp` | 验证 IPC -> scene writeback |
| 修改 | `test/CMakeLists.txt` | 注册测试目标 |

---

## `IPCTetComponent` 设计

这个组件挂在 scene 的 `GameObject` 上，作用类似 rigid body 侧的桥接组件，但它对应的是一个 tet body。

建议最小字段如下：

```cpp
class IPCTetComponent final : public Component {
private:
    std::size_t m_body_index{0};
    system::physics::ipc::TetSurfaceResult m_surface_cache{};
    rtr::utils::ObjMeshData m_mesh_cache{};

public:
    explicit IPCTetComponent(core::GameObject& owner,
                             std::size_t body_index,
                             system::physics::ipc::TetSurfaceResult surface_cache,
                             rtr::utils::ObjMeshData mesh_cache);

    std::size_t body_index() const;
    const system::physics::ipc::TetSurfaceResult& surface_cache() const;
    system::physics::ipc::TetSurfaceResult& surface_cache();

    const rtr::utils::ObjMeshData& mesh_cache() const;
    rtr::utils::ObjMeshData& mesh_cache();
};
```

### 它和 `DeformableMeshComponent` 的职责边界

这里要明确把“初始 mesh 生成”和“运行时 mesh 回写”拆开。

`IPCTetComponent` 的职责只有：

- 记录它对应 `IPCSystem` 中的哪个 tet body
- 缓存 `surface_cache`
- 缓存可原地更新的 `mesh_cache`

它**不负责**：

- 创建 `DeformableMeshComponent`
- 创建 deformable mesh resource
- 从零生成初始 render mesh

初始 mesh 的来源明确仍然是：

```text
TetBody
  -> extract_tet_surface(body)
  -> tet_to_mesh(body.geometry, surface)
  -> initial ObjMeshData
```

然后才是：

```text
initial ObjMeshData
  -> create DeformableMesh resource
  -> add DeformableMeshComponent
```

最后 `IPCTetComponent` 只把：

- `body_index`
- `surface_cache`
- `mesh_cache`

挂到同一个 `GameObject` 上，供 `sync_ipc_to_scene()` 使用。

所以正确依赖关系是：

- 初始 mesh 构建依赖 `TetBody`
- 运行时回写依赖 `IPCTetComponent + DeformableMeshComponent`
- `IPCTetComponent` 不反向依赖 `DeformableMeshComponent` 的构造

### 为什么缓存 `surface` 和 `mesh_cache`

`surface_cache` 的作用：

- 保存 `extract_tet_surface(...)` 的结果
- 避免每帧重新做表面提取
- 这一步是拓扑级缓存，初始化一次即可

`mesh_cache` 的作用：

- 每帧通过 `update_mesh_positions(mesh_cache, state.x, surface_cache)` 就地更新
- 避免每帧重新构造一个新的 `ObjMeshData`
- 这样 `sync_ipc_to_scene()` 的路径更轻，也和我们现在 `tet_mesh_convert.hpp` 的设计一致

---

## `ipc_scene_sync.hpp` 设计

最小目标只做一件事：

```cpp
sync_ipc_to_scene(core::Scene&, const IPCSystem&)
```

### 处理流程

遍历 scene 中的 active nodes：

1. 找到 `IPCTetComponent`
2. 找到同一 `GameObject` 上的 `DeformableMeshComponent`
3. 从 `ipc_system.state().x` 读取全局 DOF
4. 用 `update_mesh_positions(component.mesh_cache(), state.x, component.surface_cache())`
5. 从更新后的 `mesh_cache` 提取 positions / normals
6. `deformable->apply_deformed_surface(positions, normals)`

### 为什么这里用 `update_mesh_positions`

不建议在 `sync_ipc_to_scene()` 里每帧调一次 `tet_to_mesh(state.x, surface)`，因为那会每帧重新构造一个新的 mesh 对象。

当前实现更适合：

```text
初始化：extract_tet_surface + tet_to_mesh(rest_geometry, surface)
每帧：update_mesh_positions(mesh_cache, state.x, surface)
```

这正好和现有 `tet_mesh_convert.hpp` 的设计目标一致。

---

## `scene_physics_step.hpp` 修改

当前 `scene_physics_step.hpp` 已经在 rigid body sync 之后单独调用了 `ipc_system().step()`，这次只需要把 scene writeback 接进去。

目标顺序：

```cpp
inline void step_scene_physics(core::Scene& scene,
                               system::physics::PhysicsSystem& physics_system,
                               float delta_seconds) {
    sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
    physics_system.step(delta_seconds);
    sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());

    physics_system.ipc_system().step();
    sync_ipc_to_scene(scene, physics_system.ipc_system());
}
```

这一步的意义是把 IPC 的“求解 + scene 回写”正式纳入 framework 生命周期，而不是继续让 example 自己手动补回写逻辑。

---

## 最小示例：固定一端 block

### 文件名

建议把示例文件命名为：

```text
examples/editor/ipc_fixed_end_block_editor.cpp
```

这样比 `ipc_falling_block_editor.cpp` 更准确。

### 场景内容

最小示例只保留这些元素：

- camera
- light
- 一个 `ipc_block`
- 可选的 ground quad 作为视觉参考

这里的 ground 只做视觉参考，不参与物理。

### body 生成策略

仍然使用现有的 block generator：

```cpp
auto body = ipc::generate_tet_block(nx, ny, nz, spacing, origin);
```

建议做成一个细长一点的 block，更像 cantilever：

```cpp
auto body = ipc::generate_tet_block(
    6, 2, 2,
    0.2,
    Eigen::Vector3d{-0.6, 1.4, -0.2});
```

这样它在重力下的下垂更明显。

### 固定一端的 DBC

这次不做“固定一个单顶点”，而是固定 block 左端面上的所有顶点。

实现方式：

1. 找到 `rest_positions` 中最小的 `x`
2. 对所有满足 `abs(x_i - min_x) < eps` 的顶点，令 `fixed_vertices[i] = true`

伪代码：

```cpp
double min_x = std::numeric_limits<double>::infinity();
for (const auto& X : body.geometry.rest_positions) {
    min_x = std::min(min_x, X.x());
}

body.fixed_vertices.resize(body.geometry.rest_positions.size(), false);
for (std::size_t i = 0; i < body.geometry.rest_positions.size(); ++i) {
    if (std::abs(body.geometry.rest_positions[i].x() - min_x) < 1e-9) {
        body.fixed_vertices[i] = true;
    }
}
```

这会把这一端所有顶点的 3 个 DOF 全部固定。  
对当前 solver 和 `fixed_vertices` 设计来说，这是最自然、最稳定的最小约束形式。

---

## example 初始化流程

初始化顺序建议如下：

1. 创建 runtime / editor / scene / camera / light
2. 生成 `TetBody`
3. 设置固定端点 `fixed_vertices`
4. `extract_tet_surface(body)`
5. `tet_to_mesh(body.geometry, surface)` 生成初始 render mesh
6. 用 `ResourceManager` 创建 `DeformableMeshResource`
7. 创建 `ipc_block` GameObject
8. 挂 `DeformableMeshComponent`
9. 把 `body` 注册到 `runtime.physics_system().ipc_system()`
10. 挂 `IPCTetComponent(body_index, surface, mesh_cache)`
11. 调 `ipc_system.initialize()`
12. `runtime.run()`

这里和旧方案最大的区别是：

- example 不再自己维护 `on_post_update` 中的 mesh writeback
- writeback 统一收敛到 `sync_ipc_to_scene()`
- `IPCTetComponent` 不参与初始 mesh 创建，只保存回写缓存

### 关于注册 helper

如果示例样板代码太长，可以补一个很小的 helper 来封装注册流程，例如：

```cpp
register_ipc_tet_object(game_object, runtime, std::move(body), color);
```

这个 helper 内部只负责把下面这条流程包起来：

```text
TetBody
  -> TetSurfaceResult
  -> initial ObjMeshData
  -> DeformableMesh resource
  -> DeformableMeshComponent
  -> IPCSystem registration
  -> IPCTetComponent
```

但 helper 只是封装，不改变职责边界：

- 初始 mesh 仍然来自 `TetBody`
- `IPCTetComponent` 仍然只是桥接缓存组件

---

## 初始化数据流

```text
generate_tet_block()
    -> mark fixed end vertices
    -> extract_tet_surface(body)
    -> tet_to_mesh(body.geometry, surface)
    -> create DeformableMesh resource
    -> GameObject.add_component<DeformableMeshComponent>()
    -> ipc_system.add_tet_body(body)
    -> GameObject.add_component<IPCTetComponent>(body_index, surface, mesh_cache)
    -> ipc_system.initialize()
```

## 每个 fixed tick 的数据流

```text
step_scene_physics(...)
    -> rigid body sync in/out
    -> ipc_system.step()
    -> sync_ipc_to_scene(...)
        -> update_mesh_positions(mesh_cache, state.x, surface_cache)
        -> apply_deformed_surface(positions, normals)
    -> render
```

---

## 验收标准

### 生命周期层面

1. `IPCTetComponent` 能编译并挂到 `GameObject`
2. `scene_physics_step.hpp` 中 IPC step 后会自动调用 `sync_ipc_to_scene`
3. example 不需要自己在 `on_post_update` 手动回写 mesh

### 视觉层面

1. 程序能启动，不崩溃
2. block 初始时一端固定在空中
3. 自由端在重力下逐渐下垂
4. 固定端位置不动
5. 法线正常，没有明显黑面/白面
6. 运行 10 秒无 NaN / 爆炸

### 行为层面

1. 暂停时形变停止
2. 恢复时继续演化
3. 不需要地面接触也能稳定展示弹性响应

---

## 为什么这版比旧 demo 更合适

旧 demo 的重点是“尽快看到一个会动的 IPC block”，但它把 writeback 逻辑放在 example 自己手里，生命周期位置不对。

这版的好处是：

- 先把 framework 层真正缺失的桥接补上
- example 本身会更薄，职责更清晰
- 后续再加第二个 IPC body、scene 级管理、Inspector 展示时，不用推翻最小示例

换句话说，这次最小示例不只是“为了演示”，而是顺手把 IPC 接进 scene/runtime 的基础骨架搭好。

---

## 后续工作

这个最小示例完成之后，后续自然的扩展顺序是：

1. 给 `IPCTetComponent` 增加更多 runtime 状态或调试信息
2. 支持多个 IPC body
3. 把固定顶点从 `fixed_vertices` 升级到更丰富的约束表达
4. 再往后接 contact / barrier energy / obstacle body

当前这份 demo 计划的重点不是“做一个尽量炫的示例”，而是先把 **IPC component + scene sync + fixed-end deformable example** 这条最小闭环稳定落地。
