# Phase 4: IPCSystem 组装

## Per-Body Material Variant

### 设计动机

之前的方案是 IPCSystem 持有一个全局 `FixedCorotatedMaterial m_material`，所有 body 共用。这有两个问题：

1. **不同 body 可能需要不同材料模型**：比如一个用 FixedCorotated，另一个用 NeoHookean
2. **材料模型是 body 的固有属性**，不应该由 system 统一持有

新方案：**TetBody 自己存储一个 `TetMaterialVariant`**，每个 body 携带自己的材料模型。

### `TetMaterialVariant` 类型定义

```cpp
// energy/material_model/tet_material_variant.hpp
#pragma once

#include <variant>
#include "rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp"
// #include "rtr/system/physics/ipc/energy/material_model/tet_neo_hookean.hpp"  // Day 2+

namespace rtr::system::physics::ipc {

/// Day 1 只有一种材料，但 variant 框架已就位
using TetMaterialVariant = std::variant<
    FixedCorotatedMaterial
    // NeoHookeanMaterial,  // Day 2+
    // StVKMaterial,        // Day 2+
>;

}  // namespace rtr::system::physics::ipc
```

### TetBody 变更

```cpp
struct TetBody {
    IPCBodyInfo info{.type = IPCBodyType::Tet};
    TetGeometry geometry{};

    std::vector<double> vertex_masses{};

    double density{1000.0};
    double youngs_modulus{1e5};
    double poisson_ratio{0.3};

    TetMaterialVariant material{FixedCorotatedMaterial{}};  // 新增：per-body 材料

    std::vector<bool> fixed_vertices{};
    // ...
};
```

`youngs_modulus` 和 `poisson_ratio` 仍然留在 TetBody 上，因为它们是**物理参数**（所有材料模型都需要），而 `material` 是**本构模型**（同样的 $E$、$\nu$ 在不同模型下给出不同的应力-应变关系）。

### MaterialEnergy 的 Variant Dispatch

`MaterialEnergy` 的模板参数从编译期固定变为通过 `std::visit` 在 variant 上运行时 dispatch：

```cpp
// 新增：variant-aware 版本
namespace material_energy_variant {

inline double compute_energy(const TetBody& body, const Eigen::VectorXd& x) {
    return std::visit([&](const auto& material) {
        return MaterialEnergy<std::decay_t<decltype(material)>>::compute_energy(
            {.body = body, .x = x, .material = material});
    }, body.material);
}

inline void compute_gradient(const TetBody& body, const Eigen::VectorXd& x,
                             Eigen::VectorXd& gradient) {
    std::visit([&](const auto& material) {
        MaterialEnergy<std::decay_t<decltype(material)>>::compute_gradient(
            {.body = body, .x = x, .material = material}, gradient);
    }, body.material);
}

inline void compute_hessian_triplets(const TetBody& body, const Eigen::VectorXd& x,
                                     std::vector<Eigen::Triplet<double>>& triplets) {
    std::visit([&](const auto& material) {
        MaterialEnergy<std::decay_t<decltype(material)>>::compute_hessian_triplets(
            {.body = body, .x = x, .material = material}, triplets);
    }, body.material);
}

}  // namespace material_energy_variant
```

**关键点**：`std::visit` 的 lambda 接收 `const auto& material`，编译器为 variant 中的每个类型生成一份特化——**内循环仍然是模板 inline，没有虚函数开销**。运行时 dispatch 只发生一次（在 variant 的 type index 上做 switch），之后整个 per-tet 循环都是单态内联的。

### 性能分析

```
std::visit 开销（per body per step）:
  1 次 variant index 检查 → 分支预测几乎必中（同一 body 的类型不会变）
  之后全部是模板内联的 per-tet 循环

vs 之前的方案（IPCSystem 持有 m_material）:
  0 次 dispatch，纯编译期

差异：每 body 多一次可预测分支。对于内循环 O(T) 的 tet 遍历来说可忽略。
```

## File: `core/ipc_system.hpp`

### 接口

```cpp
namespace rtr::system::physics::ipc {

struct IPCConfig {
    double dt{0.01};                      // 固定时间步
    Eigen::Vector3d gravity{0.0, -9.81, 0.0};
    NewtonSolverParams solver_params{};
};

class IPCSystem {
public:
    explicit IPCSystem(IPCConfig config);

    // Body 注册
    void add_tet_body(TetBody body);
    // void add_obstacle_body(ObstacleBody body);  // Day 3

    // 初始化：组装全局状态、预计算
    void initialize();

    // 单步推进
    void step();

    // 状态访问
    const IPCState& state() const;
    const TetBody& tet_body(std::size_t index) const;

    // 顶点回读（用于 scene write-back）
    std::vector<Eigen::Vector3d> get_body_positions(std::size_t body_index) const;

private:
    IPCConfig m_config;
    IPCState m_state;
    std::vector<TetBody> m_tet_bodies;
    // std::vector<ObstacleBody> m_obstacle_bodies;  // Day 3

    // 不再持有 m_material —— 材料模型由每个 TetBody 自己携带

    Eigen::VectorXd m_x_hat;  // 预测位置
    std::vector<bool> m_free_dof_mask;  // per-DOF, 从 vertex_constraints 展开

    void compute_x_hat();
    void build_free_dof_mask();

    // 总能量装配
    double compute_total_energy(const Eigen::VectorXd& x);
    void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient);
    void compute_total_hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets);
};

}
```

### initialize() 做的事

1. 对每个 TetBody 调 `precompute()`
2. 计算全局顶点总数，resize `IPCState`
3. 把每个 body 的 rest positions 写入 `state.x` 和 `state.x_prev`
4. 把每个 body 的质量写入 `state.mass_diag`
5. 速度初始化为 0
6. 构建 `m_free_dof_mask`

### step() 做的事

```
1. state.x_prev = state.x
2. compute_x_hat()
3. newton_solve(state, x_hat, free_dof_mask, ...)
4. state.v = (state.x - state.x_prev) / dt
5. 对 fixed DOFs: state.v[i] = 0  （按 free_dof_mask 逐 DOF 清零）
   // stick vertex: 3 个分量全清零
   // slip vertex: 只清零被约束的轴分量，保留自由轴的速度
```

### compute_x_hat()

$\hat{x}$ 是纯惯性预测位置，不含外力：

```cpp
// x_hat = x_prev + dt * v  （纯惯性，重力由 GravityEnergy 独立处理）
m_x_hat = m_state.x_prev + m_config.dt * m_state.v;
```

### 总能量装配

通过 `material_energy_variant` 命名空间的帮助函数，每个 body 自动 dispatch 到自己的材料模型：

```cpp
double compute_total_energy(const Eigen::VectorXd& x) {
    double E = 0.0;
    E += InertialEnergy::compute_energy(x, m_x_hat, m_state.mass_diag, m_config.dt);
    E += GravityEnergy::compute_energy(x, m_state.mass_diag, m_config.gravity);
    for (const auto& body : m_tet_bodies) {
        E += material_energy_variant::compute_energy(body, x);  // variant dispatch
    }
    // Day 3: += BarrierEnergy::compute_energy(...)
    return E;
}
```

gradient 同理，三项用 `+=` 模式累加：

```cpp
void compute_total_gradient(const Eigen::VectorXd& x, Eigen::VectorXd& gradient) {
    gradient.setZero();
    InertialEnergy::compute_gradient(x, m_x_hat, m_state.mass_diag, m_config.dt, gradient);
    GravityEnergy::compute_gradient(m_state.mass_diag, m_config.gravity, gradient);
    for (const auto& body : m_tet_bodies) {
        material_energy_variant::compute_gradient(body, x, gradient);  // variant dispatch
    }
}
```

Hessian 只有 inertial + elastic 贡献（gravity 是线性能量，Hessian 为零）：

```cpp
void compute_total_hessian(const Eigen::VectorXd& x, std::vector<Eigen::Triplet<double>>& triplets) {
    triplets.clear();
    InertialEnergy::compute_hessian_triplets(m_state.mass_diag, m_config.dt, triplets);
    // GravityEnergy: 无 Hessian 贡献（线性能量）
    for (const auto& body : m_tet_bodies) {
        material_energy_variant::compute_hessian_triplets(body, x, triplets);  // variant dispatch
    }
}
```

**对比旧方案**：原来是 `MaterialEnergy<FixedCorotatedMaterial>::compute_energy(body, x, m_material)`，material 类型在 IPCSystem 编译时固定。现在是 `material_energy_variant::compute_energy(body, x)`，material 从 body.material variant 中取，调用方不需要知道具体类型。

### Day 3 扩展点

Day 3 加 contact 时，在总能量装配里加：

```cpp
E += kappa * BarrierEnergy::compute_energy(collision_constraints, x, dhat);
```

梯度和 Hessian 同理。CCD 在 line search 里加 `alpha_max` 限制。

### 与现有系统的关系

Day 1 将 IPCSystem 接入 PhysicsSystem，并通过 scene sync 层打通 IPC → DeformableMeshComponent 的渲染回写。

---

## Phase 4b: IPCSystem 接入 PhysicsSystem

### PhysicsSystem 扩展

```cpp
// physics_system.hpp
#pragma once
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"

namespace rtr::system::physics {

class PhysicsSystem {
private:
    RigidBodyWorld m_rigid_body_world{};
    ipc::IPCSystem m_ipc_system{ipc::IPCConfig{}};

public:
    RigidBodyWorld& rigid_body_world() { return m_rigid_body_world; }
    const RigidBodyWorld& rigid_body_world() const { return m_rigid_body_world; }

    ipc::IPCSystem& ipc_system() { return m_ipc_system; }
    const ipc::IPCSystem& ipc_system() const { return m_ipc_system; }

    void step(float delta_seconds) {
        m_rigid_body_world.step(delta_seconds);
        // IPC 使用固定时间步，由外部 fixed_tick 驱动，不在这里 step
        // IPC step 在 scene_physics_step 中独立调用
    }
};

}  // namespace rtr::system::physics
```

说明：IPC 的 `step()` 不放在 `PhysicsSystem::step()` 里，因为：
1. IPC 使用固定 dt（如 0.01s），与刚体物理的 `delta_seconds` 可能不同
2. IPC step 需要在 scene sync 之间调用（先 sync scene → step → sync back）
3. 保持 `PhysicsSystem::step()` 不变，刚体部分零侵入

### Scene Sync 层

类比 `rigid_body_scene_sync.hpp`，新增 `ipc_scene_sync.hpp`：

```cpp
// src/rtr/framework/integration/physics/ipc_scene_sync.hpp
#pragma once

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_body_component.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::framework::integration::physics {

// Phase 1: Scene → IPC（Day 1 初始化时调用一次）
// Day 1 的 IPC body 是静态注册的，不需要每帧 sync scene → physics。
// 这个函数预留给未来动态添加/删除 IPC body 的场景。

// Phase 2: IPC → Scene（每帧 step 后调用）
inline void sync_ipc_to_scene(core::Scene& scene,
                              const system::physics::ipc::IPCSystem& ipc_system) {
    const auto active_nodes = scene.scene_graph().active_nodes();
    for (const auto id : active_nodes) {
        auto* game_object = scene.find_game_object(id);
        if (game_object == nullptr) {
            continue;
        }

        auto* ipc_body = game_object->get_component<component::IPCBodyComponent>();
        if (ipc_body == nullptr || !ipc_body->enabled()) {
            continue;
        }

        auto* deformable = game_object->get_component<component::DeformableMeshComponent>();
        if (deformable == nullptr) {
            continue;
        }

        // 从 IPCSystem 读取当前位置
        const auto& state = ipc_system.state();
        const auto& surface = ipc_body->surface_cache();

        // 更新 ObjMeshData（positions + normals）
        auto mesh = system::physics::ipc::tet_to_mesh(state.x, surface);

        // 提取 positions 和 normals
        std::vector<pbpt::math::Vec3> positions;
        std::vector<pbpt::math::Vec3> normals;
        positions.reserve(mesh.vertices.size());
        normals.reserve(mesh.vertices.size());
        for (const auto& v : mesh.vertices) {
            positions.push_back(v.position);
            normals.push_back(v.normal);
        }

        // 写回 DeformableMeshComponent → 标记 cpu_dirty → GPU 自动同步
        deformable->apply_deformed_surface(positions, normals);
    }
}

}  // namespace rtr::framework::integration::physics
```

### IPCBodyComponent

挂载在 GameObject 上的 ECS 组件，桥接 scene 和 IPCSystem：

```cpp
// src/rtr/framework/component/physics/ipc/ipc_body_component.hpp
#pragma once

#include "rtr/framework/component/component.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::framework::component {

class IPCBodyComponent final : public Component {
private:
    std::size_t m_body_index{0};  // IPCSystem 中的 body 索引
    system::physics::ipc::TetSurfaceResult m_surface_cache{};

public:
    explicit IPCBodyComponent(core::GameObject& owner,
                              std::size_t body_index,
                              system::physics::ipc::TetSurfaceResult surface)
        : Component(owner),
          m_body_index(body_index),
          m_surface_cache(std::move(surface)) {}

    std::size_t body_index() const { return m_body_index; }
    const system::physics::ipc::TetSurfaceResult& surface_cache() const { return m_surface_cache; }
};

}  // namespace rtr::framework::component
```

### scene_physics_step 扩展

```cpp
// scene_physics_step.hpp 更新
inline void step_scene_physics(core::Scene& scene,
                               system::physics::PhysicsSystem& physics_system,
                               float delta_seconds) {
    // 刚体 sync（保持不变）
    sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
    physics_system.step(delta_seconds);
    sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());

    // IPC sync（新增）
    // Day 1: scene→IPC 不需要每帧同步（body 是静态注册的）
    physics_system.ipc_system().step();
    sync_ipc_to_scene(scene, physics_system.ipc_system());
}
```

### 数据流

```
初始化：
  generate_tet_block() → TetBody
  extract_tet_surface(body) → TetSurfaceResult（缓存）
  tet_to_mesh(geometry, surface) → ObjMeshData
  ResourceManager.create_from_data<DeformableMeshResourceKind>(mesh) → DeformableMeshHandle
  GameObject.add_component<DeformableMeshComponent>(handle)
  GameObject.add_component<IPCBodyComponent>(body_index, surface)
  ipc_system.add_tet_body(body)
  ipc_system.initialize()

每帧（fixed_tick）：
  ipc_system.step()
      → Newton solve → state.x 更新
  sync_ipc_to_scene()
      → tet_to_mesh(state.x, surface) → ObjMeshData
      → apply_deformed_surface(positions, normals)
      → ResourceManager 标记 cpu_dirty
  渲染：
      → require_gpu() 检测 cpu_dirty → 上传 GPU
      → draw
```

---

## Phase 4c: IPC Demo Example

### File: `examples/editor/ipc_falling_block_editor.cpp`

一个自由落体的弹性方块，底面用 slip DBC 约束 y 轴（模拟地面接触），展示完整的 IPC → 渲染管线。

```cpp
#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/render/forward_editor_pipeline.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_body_component.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

int main() {
    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;

    rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
        .window_width = kWidth, .window_height = kHeight,
        .window_title = "RTR IPC Falling Block Demo"});

    // --- editor setup (同 deformable_mesh_editor) ---
    auto editor_host = std::make_shared<rtr::editor::EditorHost>(runtime);
    editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
    editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
    editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
    editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
    editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());

    auto editor_pipeline = std::make_unique<rtr::editor::render::ForwardEditorPipeline>(
        runtime.renderer().build_pipeline_runtime(), editor_host);
    rtr::editor::bind_input_capture_to_editor(runtime.input_system(), *editor_pipeline);
    runtime.set_pipeline(std::move(editor_pipeline));

    auto& scene = runtime.world().create_scene("ipc_scene");

    // --- camera ---
    auto& camera_go = scene.create_game_object("main_camera");
    auto& camera = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
    camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
    camera.set_active(true);
    camera_go.node().set_local_position({0.0f, 2.0f, 8.0f});
    camera_go.add_component<rtr::framework::component::FreeLookCameraController>(
        runtime.input_system().state());
    camera.camera_look_at_point_world(pbpt::math::Vec3{0.0, 0.0, 0.0});

    // --- light ---
    auto& light_go = scene.create_game_object("main_light");
    light_go.node().set_local_position({3.0f, 6.0f, 5.0f});
    auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
    point_light.set_color({1.0f, 0.95f, 0.85f});
    point_light.set_intensity(60.0f);
    point_light.set_range(25.0f);

    // --- ground plane (visual only) ---
    auto& ground_go = scene.create_game_object("ground");
    const auto ground_mesh = runtime.resource_manager()
        .create_from_relative_path<rtr::resource::MeshResourceKind>("models/colored_quad.obj");
    (void)ground_go.add_component<rtr::framework::component::StaticMeshComponent>(
        runtime.resource_manager(), ground_mesh, pbpt::math::Vec4{0.3f, 0.3f, 0.32f, 1.0f});
    ground_go.node().set_local_position({0.0f, 0.0f, 0.0f});
    ground_go.node().set_local_rotation(
        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}));
    ground_go.node().set_local_scale({10.0f, 10.0f, 1.0f});

    // === IPC setup ===
    namespace ipc = rtr::system::physics::ipc;

    // 1. 生成 tet block（3×3×3 网格，spacing=0.3，起始于 y=2 处）
    auto body = ipc::generate_tet_block(3, 3, 3, 0.3, Eigen::Vector3d{-0.45, 2.0, -0.45});
    body.material = ipc::FixedCorotatedMaterial{};  // per-body 材料（Day 2+ 可换成其他模型）

    // 2. 底面 y-slip DBC：y ≈ 2.0 的顶点约束 y 轴
    //    （实际 demo 中不约束，让方块自由落体；或约束底面模拟平面接触）
    //    这里演示自由落体，不设 DBC

    // 3. 提取表面 + 构建初始渲染 mesh
    auto surface = ipc::extract_tet_surface(body);
    auto initial_mesh = ipc::tet_to_mesh(body.geometry, surface);

    // 4. 注册到 ResourceManager 作为 DeformableMesh
    //    ResourceManager::create<Kind>(cpu_data) 直接从内存数据创建资源
    auto mesh_handle = runtime.resource_manager()
        .create<rtr::resource::DeformableMeshResourceKind>(std::move(initial_mesh));

    // 5. 创建 GameObject + 组件
    auto& block_go = scene.create_game_object("ipc_block");
    auto& deformable = block_go.add_component<rtr::framework::component::DeformableMeshComponent>(
        runtime.resource_manager(), mesh_handle, pbpt::math::Vec4{0.7f, 0.4f, 0.3f, 1.0f});
    (void)block_go.add_component<rtr::framework::component::IPCBodyComponent>(
        0, surface);

    // 6. 注册 body 到 IPCSystem 并初始化
    auto& ipc_system = runtime.physics_system().ipc_system();
    ipc_system.add_tet_body(std::move(body));
    ipc_system.initialize();

    // --- runtime callbacks ---
    runtime.set_callbacks(rtr::app::RuntimeCallbacks{
        .on_post_update =
            [&](rtr::app::RuntimeContext& ctx) {
                editor_host->begin_frame(rtr::editor::EditorFrameData{
                    .frame_serial  = ctx.frame_serial,
                    .delta_seconds = ctx.delta_seconds,
                    .paused        = ctx.paused,
                });
            },
        .on_pre_render =
            [](rtr::app::RuntimeContext& ctx) {
                if (ctx.input_system.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                    ctx.renderer.window().close();
                }
            }});

    runtime.run();
    return 0;
}
```

### 关键设计决策

| 决策 | 理由 |
|------|------|
| `IPCBodyComponent` 缓存 `TetSurfaceResult` | 表面提取是 $O(T)$ 操作，初始化时做一次即可 |
| `sync_ipc_to_scene` 每帧重建 `ObjMeshData` | Day 1 简单方案；Day 2+ 可用 `update_mesh_positions` 就地更新 |
| IPC step 在 `step_scene_physics` 中调用 | 复用现有 fixed_tick 驱动，保证 sync → step → sync 顺序 |
| demo 不用 `SineWaveDeformer` 组件模式 | IPC 的驱动是物理引擎，不是组件 `on_fixed_update`；sync 层负责回写 |
| `create_from_data` 创建 DeformableMesh | 因为 tet mesh 是程序化生成的，不从文件加载 |

### CMake

```cmake
# examples/CMakeLists.txt 新增
add_executable(ipc_falling_block_editor editor/ipc_falling_block_editor.cpp)
target_link_libraries(ipc_falling_block_editor PRIVATE rtr::runtime rtr::editor)
```

### 验收标准

- `initialize()` 后 `state()` 有正确大小
- `step()` 一次后顶点位置有变化
- stick DBC vertices 保持完全固定
- slip DBC vertices 在约束轴上不动，自由轴可以移动
- 多步 step 后不出 NaN
- **方块在窗口中可见，随时间下落**
- **DeformableMeshComponent 的顶点每帧更新**（CPU dirty → GPU sync 正常工作）
- **方块形变时法线正确更新**（光照无异常）

### Day 2+ 优化方向

1. `sync_ipc_to_scene` 改用 `update_mesh_positions` 就地更新（避免每帧重建 ObjMeshData）
2. `IPCBodyComponent` 存储 body_index → 支持多 body
3. `sync_scene_to_ipc` 支持运行时动态添加/删除 body
4. `IPCConfig` 从 Inspector 面板可调（dt、gravity、solver params）
