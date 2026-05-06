# libpgo 集成到 RTR2 的路线图

## 核心架构决策

**所有物理仿真算法在 libpgo 中实现。RTR2 的 IPCSystem 是对 libpgo 的薄转发层。**

```
┌────────────────────────────────────────────────┐
│  RTR2                                          │
│                                                │
│  IPCSystem (薄转发层)                           │
│    ├─ scene data → libpgo data 转换             │
│    ├─ 调用 libpgo 仿真 API                      │
│    └─ libpgo 结果 → scene/renderer 回写         │
│                                                │
│  RigidBodyWorld (保留现有，独立于 libpgo)         │
│  Editor / Renderer / Scene Sync                │
└────────────────────┬───────────────────────────┘
                     │ C++ API 调用
┌────────────────────▼───────────────────────────┐
│  libpgo (物理引擎)                              │
│                                                │
│  ┌─ FEM ─────────────────────────────────────┐ │
│  │  Tet FEM (现有) + Cubic/Hex FEM (新增)    │ │
│  │  Shell FEM (现有)                         │ │
│  │  多材料：SNH, NeoHookean, StVK, ...       │ │
│  └───────────────────────────────────────────┘ │
│  ┌─ IPC Contact (新增) ─────────────────────┐ │
│  │  Barrier energy + CCD + Self-contact      │ │
│  │  Feasibility-aware line search            │ │
│  └───────────────────────────────────────────┘ │
│  ┌─ Solver ──────────────────────────────────┐ │
│  │  Newton + BE/TRBDF2 time integrator       │ │
│  │  Sparse linear solve (Eigen/MKL)          │ │
│  └───────────────────────────────────────────┘ │
└────────────────────────────────────────────────┘
```

这意味着：
- rtr2 **不实现**任何物理求解算法
- rtr2 现有的 `ipc/energy/`, `ipc/solver/`, `ipc/geometry/` 将被移除
- 新功能（cubic mesh FEM、IPC）在 libpgo 仓库中开发
- rtr2 只关心数据搬运和渲染

---

## 与 libpgo 开发计划的对齐

libpgo 自身有一份 12 天 3-Phase 执行计划（见 `libpgo/papers/plan/overview.md` 和 `implementation.md`）：

| libpgo Phase | 内容 | 对 rtr2 的影响 |
|-------------|------|---------------|
| Phase 1 (Day 1-6) | 打通 cubic/hex FEM 最小主线 | rtr2 可开始对接 libpgo 的 tet FEM |
| Phase 2 (Day 7-10) | 接入 IPC prototype (barrier + CCD) | rtr2 的 IPC 转发层可对接接触管线 |
| Phase 3 (Day 11-12) | 接口固化、验证、example | rtr2 对接 libpgo 的稳定 API |

**rtr2 的集成工作可以在 libpgo Phase 1 进行期间就启动。** 最初只需 libpgo 的现有 tet FEM 能力（已经可用），不依赖 cubic FEM 和 IPC 完成。

---

## RTR2 侧的实施阶段

### Phase R0: 构建集成

**目标：** rtr2 能编译并链接 libpgo。

**任务：**

1. **引入 libpgo 到 `external/libpgo/`**
   - 作为 git submodule
   - 顶层 `CMakeLists.txt` 添加 `add_subdirectory(external/libpgo)`
   - 最小配置构建：`PGO_USE_MKL=0 PGO_ENABLE_FULL=0`

2. **依赖协调**
   - 共享依赖（Eigen, TBB, fmt）统一由 rtr2 Conan 提供
   - CMake 最低版本升级到 3.28

3. **链接 libpgo 的必要 target**
   - `mesh`, `volumetricMesh`, `solidDeformationModel`, `simulation`, `contact`, `nonlinearOptimization`, `eigenSupport`

**交付标准：**
- rtr2 编译通过，能 `#include` 并调用 libpgo API
- CI 通过

**时机：** 可立即开始，与 libpgo Phase 1 并行。

---

### Phase R1: IPCSystem 重构为转发层

**目标：** 将 rtr2 的 IPCSystem 从自有实现重构为调用 libpgo。

**任务：**

1. **确定 libpgo 的调用接口**
   - 如果 libpgo Phase 3 已产出稳定 API（如 `runSimCore`），直接使用
   - 如果尚未稳定，先直接调用 libpgo 内部类（`TimeIntegrator`, `DeformationModelManager` 等），待 API 稳定后再切换
   - 关键调用链：
     - 创建 `TetMesh` / `CubicMesh` → 创建 `SimulationMesh` → 设置 `DeformationModel` → 创建 `TimeIntegrator` → 调用 `step()`

2. **移除 rtr2 自有物理代码**
   - 移除：
     - `ipc/energy/` — inertial_energy, gravity_energy, material_energy, material_model/
     - `ipc/solver/` — newton_solver, line_search
     - `ipc/geometry/` — distance 计算
   - 保留并改造：
     - `ipc/core/ipc_system.hpp` — 内部持有 libpgo 仿真对象的 handle
     - `ipc/model/` — body 定义简化为 libpgo body 的薄包装
     - `ipc/contact/` — 不再需要，碰撞由 libpgo 内部管理

3. **新 IPCSystem 实现**
   ```
   IPCSystem
     ├─ m_world: libpgo 的仿真上下文（SimulationMesh, TimeIntegrator, ContactHandler 等）
     ├─ create_tet_body():   rtr2 mesh → libpgo TetMesh → 注册到仿真
     ├─ create_cubic_body(): rtr2 mesh → libpgo CubicMesh → 注册到仿真
     ├─ create_obstacle():   rtr2 mesh → libpgo TriMeshGeo → 注册为障碍物
     ├─ step(dt):            调用 libpgo 的 time integrator step
     └─ get_positions(id):   从 libpgo 读回变形后的顶点位置
   ```

4. **Scene sync 适配**
   - 现有 `ipc_scene_sync` 框架保留
   - 数据来源从 rtr2 自有 `IPCState` 改为从 libpgo 读取
   - `pbpt::math` ↔ `Eigen` 转换层保留（已有）

**交付标准：**
- IPCSystem 内部无任何求解代码，全部委托给 libpgo
- 现有 tet body 仿真场景在新架构下正常运行
- 无穿透接触（当 libpgo IPC prototype 可用时）

**时机：**
- 基础 tet FEM 转发：libpgo Phase 1 期间即可开始
- IPC 接触转发：等 libpgo Phase 2 完成后对接

---

### Phase R2: Cubic Body 支持

**目标：** rtr2 能创建、仿真和渲染 cubic/hex mesh body。

**前置条件：** libpgo Phase 1 完成（cubic FEM 主线跑通）。

**任务：**

1. **Cubic mesh 导入**
   - 支持 `.veg` 格式的 hex mesh 加载（libpgo 已支持此格式）
   - 或从表面 mesh 通过 voxelization 生成

2. **CubicBodyComponent**
   - 类似现有 `TetBodyComponent`
   - 在 `IPCSystem::create_cubic_body()` 中调用 libpgo 的 cubic mesh FEM 路径

3. **渲染同步**
   - hex body 表面提取 → 三角网格 → Vulkan renderer
   - libpgo 已有 `generateSurfaceMesh` 支持

**交付标准：**
- 场景中可放置 cubic mesh body
- 仿真结果正确渲染
- hex + tet body 可共存

---

### Phase R3: IPC Contact 对接

**目标：** 当 libpgo 的 IPC prototype 可用后，rtr2 自动获得接触能力。

**前置条件：** libpgo Phase 2 完成。

**任务：**

1. **暴露 IPC 参数到 IPCConfig**
   - `contact_model`: `penalty` | `ipc`（对应 libpgo 的 contact model 切换）
   - `dhat`, `kappa`, `friction_coeff`

2. **转发接触配置**
   - 在 IPCSystem 初始化时将接触参数传递给 libpgo
   - 选择 IPC path 时，libpgo 内部的 barrier + CCD + feasibility-aware line search 自动生效

3. **Editor 暴露**
   - ImGui 面板中可切换 penalty / IPC
   - 可调 dhat, kappa 等参数

**交付标准：**
- 从 rtr2 editor 中可切换 penalty 和 IPC 接触模型
- IPC 模式下 tet/cubic body 落地不穿透
- 参数可实时调整

---

### Phase R4: Debug 可视化与诊断

**目标：** 在 rtr2 editor 中提供物理仿真调试工具。

**任务：**
- 接触点/接触对高亮（需要 libpgo 暴露 contact info）
- 碰撞网格线框渲染
- Newton 迭代收敛曲线（从 libpgo 读取 solver stats）
- 能量分项实时显示
- 固定顶点高亮

**时机：** 可在任何阶段逐步添加。

---

## 执行时间线

```
libpgo 开发:
  Phase 1 (Day 1-6)    Phase 2 (Day 7-10)    Phase 3 (Day 11-12)
  ═══════════════════   ═══════════════════    ═══════════════════
  cubic/hex FEM         IPC prototype          接口固化
        │                      │                      │
rtr2 集成:                     │                      │
  Phase R0 ─────────→ Phase R1 (tet 转发) ──────────────→
                              │                      │
                        Phase R2 (cubic body) ←───────
                                              Phase R3 (IPC 对接)
                                              Phase R4 (debug)
```

**最小可用路径：** R0 → R1（仅 tet 转发）

这一步完成后，rtr2 就从自有物理代码切换到 libpgo 后端，后续所有 libpgo 的新功能（cubic FEM、IPC、friction 等）会自动对 rtr2 可用，只需要在转发层做薄适配。

---

## rtr2 现有代码的处置

| 现有代码 | 处置 |
|---------|------|
| `ipc/core/ipc_state.hpp` | **改造** — 简化为持有 libpgo handle |
| `ipc/core/ipc_system.hpp` | **改造** — 重写为转发层 |
| `ipc/model/tet_body.hpp` | **改造** — 简化为 libpgo TetMesh 的薄包装 |
| `ipc/model/obstacle_body.hpp` | **改造** — 简化为 libpgo obstacle 的薄包装 |
| `ipc/model/mesh_tet_converter/` | **保留** — mesh ↔ tet 转换仍需要 |
| `ipc/energy/` (全部) | **移除** — 能量计算由 libpgo 负责 |
| `ipc/solver/` (全部) | **移除** — 求解由 libpgo 负责 |
| `ipc/geometry/` (全部) | **移除** — 距离计算由 libpgo 负责 |
| `ipc/contact/collision_mesh.hpp` | **移除** — 碰撞检测由 libpgo 负责 |
| `ipc/contact/collision_candidates.hpp` | **移除** — 碰撞候选由 libpgo 负责 |

---

## 依赖与风险

| 风险 | 缓解 |
|------|------|
| libpgo API 未稳定 | Phase R1 先直接调用内部类，待 libpgo Phase 3 固化后再切 |
| 共享依赖版本冲突 (Eigen, TBB, fmt) | 统一由 rtr2 Conan 提供，libpgo 通过 CMake alias 复用 |
| CMake 版本 (rtr2: 3.27, libpgo: 3.28) | 升级 rtr2 到 3.28 |
| libpgo cubic FEM 延期 | Phase R1/R2 解耦，先用 tet FEM 验证转发层 |
| 性能（libpgo 不如手写内核） | libpgo 已有 TBB 并行 + 可选 MKL Pardiso，性能应足够 |
