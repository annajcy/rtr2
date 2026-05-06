# RTR2 Editor Mode / Game Mode Plan

## 目标

为 RTR2 编辑器增加类似商业游戏引擎（Unity / Unreal）的 **Editor Mode ↔ Game Mode** 切换机制，以及 **Play / Pause / Stop / Step** 控制，使编辑器具备完整的"编辑 → 预览 → 调试"工作流。

---

## 现状分析

### 已有能力

| 能力 | 位置 | 说明 |
|------|------|------|
| `m_paused` | `AppRuntime` | 暂停时跳过 fixed_tick / tick / late_tick，渲染和输入继续 |
| `EditorFrameData::paused` | `editor_types.hpp` | 将暂停状态传递给面板 |
| `StatsPanel` | `stats_panel.hpp` | 已显示 paused 状态文本 |
| Panel 系统 | `IEditorPanel` | 注册式面板，共享 `EditorContext` |
| Callback 系统 | `RuntimeCallbacks` | on_startup / on_input / on_pre_update / on_post_update / on_pre_render / on_post_render / on_shutdown |

### 缺失部分

- **无模式概念**：没有 Editor Mode / Game Mode 区分，编辑器始终处于同一状态
- **无场景快照 / 恢复**：进入 Game Mode 后无法回退到编辑时的场景状态
- **无 Play / Pause / Stop UI 控件**：暂停只能通过代码调用 `set_paused()`
- **无 Step 功能**：无法逐帧推进模拟
- **无 Toolbar 面板**：没有 Unity 风格的顶部工具栏

---

## 设计原则

### 1. 模式是编辑器概念，不是运行时概念

`AppRuntime` 不需要知道"Editor Mode"和"Game Mode"的存在。模式切换的本质是：

- **Editor Mode**：世界暂停，用户通过 gizmo / inspector 编辑场景
- **Game Mode**：世界运行，用户在 scene view 中预览交互结果

`AppRuntime` 只需要暴露 `set_paused()` 和 `step_one_frame()` 即可。模式语义完全在编辑器层管理。

### 2. 场景快照是 Game Mode 的安全网

进入 Game Mode 时必须保存场景快照，退出时恢复。否则"Play 一下看看效果"会永久修改场景数据，这在任何商业引擎中都是不可接受的。

快照粒度选择：

- **v0**：序列化所有 transform + physics state（最小可用）
- **v1**：完整 world 序列化 / 反序列化（需要 world 具备此能力）

### 3. 输入路由随模式切换

- **Editor Mode**：鼠标 / 键盘优先路由到 ImGui（面板 / gizmo），scene view 仅处理相机控制和拾取
- **Game Mode**：scene view 获取输入焦点，可选择隐藏编辑器 UI overlay 或保留最小调试 HUD

### 4. 渐进式实施

分三个阶段，每个阶段都产出可用功能：

- **Phase A**：Toolbar + Play / Pause / Stop UI（不含场景快照）
- **Phase B**：场景快照 / 恢复 + 完整 Editor ↔ Game 模式切换
- **Phase C**：Step 逐帧 + 高级调试功能

---

## Phase A — Toolbar 与基础播放控制

### 目标

在编辑器顶部增加工具栏面板，提供 Play / Pause / Stop 按钮，连接到现有的 `AppRuntime::set_paused()`。

### A.1 EditorMode 枚举

**文件**：`src/rtr/editor/core/editor_types.hpp`

```cpp
enum class EditorMode : std::uint8_t {
    Edit,   // 场景暂停，可编辑
    Play,   // 场景运行
    Pause,  // 场景暂停，但处于 Game Mode（可恢复 Play）
};
```

将 `EditorMode` 加入 `EditorFrameData`，替代单独的 `paused` bool。

### A.2 ToolbarPanel

**新文件**：`src/rtr/editor/panel/toolbar_panel.hpp`

职责：
- 渲染居中的 ▶ Play / ⏸ Pause / ⏹ Stop 按钮
- 按钮状态根据当前 `EditorMode` 高亮 / 禁用
- 点击时通过 `EditorServices` 回调通知宿主切换模式

UI 布局参考：
```
┌─────────────────────────────────────────────────────────┐
│  [▶ Play]  [⏸ Pause]  [⏹ Stop]          FPS: 60       │
├─────────────────────────────────────────────────────────┤
│ Hierarchy │           Scene View           │ Inspector  │
│           │                                │            │
│           │                                │            │
│           │                                │            │
├───────────┴────────────────────────────────┴────────────┤
│                       Logger / Stats                     │
└─────────────────────────────────────────────────────────┘
```

### A.3 EditorServices 扩展

**文件**：`src/rtr/editor/core/editor_types.hpp`

在 `EditorServices` 中增加模式切换回调：

```cpp
struct EditorServices {
    // ... existing callbacks ...
    std::function<void(EditorMode)> on_mode_change;
};
```

宿主（quickstart / app）在收到回调后调用 `AppRuntime::set_paused()` 等方法。

### A.4 快捷键绑定

| 快捷键 | 动作 |
|--------|------|
| `Ctrl+P` / `Cmd+P` | Play / Resume |
| `Ctrl+Shift+P` / `Cmd+Shift+P` | Pause |
| `Escape` | Stop（回到 Editor Mode） |

快捷键在 `ToolbarPanel::on_imgui()` 中通过 ImGui 的 `IsKeyPressed` 检测。

### A.5 AppRuntime 最小扩展

**文件**：`src/rtr/app/app_runtime.hpp`

新增：

```cpp
/// 执行恰好一个 fixed tick + 一个 frame tick，然后重新暂停
void step_single_frame();
```

用于 Phase C 的逐帧调试，但接口在 Phase A 即可预留。

### Checklist — Phase A

- [ ] 定义 `EditorMode` 枚举，更新 `EditorFrameData`
- [ ] 实现 `ToolbarPanel`，渲染 Play / Pause / Stop 按钮
- [ ] 在 `EditorServices` 中增加 `on_mode_change` 回调
- [ ] 在 `EditorHost` 中将 toolbar 固定在顶部（非 dockable）
- [ ] quickstart 示例中连接 `on_mode_change` → `AppRuntime::set_paused()`
- [ ] 添加快捷键支持
- [ ] `StatsPanel` 显示当前模式文本（Edit / Play / Pause）

---

## Phase B — 场景快照与模式切换

### 目标

进入 Game Mode 时保存场景状态，退出时恢复，实现安全的"试玩 → 复原"工作流。

### B.1 SceneSnapshot

**新文件**：`src/rtr/editor/core/scene_snapshot.hpp`

```cpp
struct SceneSnapshot {
    // v0: 最小快照 — transform + rigid body state
    struct GameObjectState {
        GameObjectId id;
        Transform transform;
        std::optional<RigidBodyState> rigid_body;
        std::optional<IpcSoftBodyState> soft_body;
    };
    std::vector<GameObjectState> objects;
};
```

接口：

```cpp
SceneSnapshot capture_snapshot(const World& world, SceneId scene);
void restore_snapshot(World& world, SceneId scene, const SceneSnapshot& snapshot);
```

### B.2 模式切换流程

```
                    Play
   ┌──────────┐  ────────►  ┌──────────┐
   │          │              │          │
   │  Editor  │              │  Playing │
   │  Mode    │  ◄────────   │  Mode    │
   │          │    Stop      │          │
   └──────────┘              └─────┬────┘
                                   │
                              Pause│  ▲ Resume
                                   ▼  │
                             ┌─────────┐
                             │ Paused  │
                             │ (Game)  │
                             └─────────┘
```

**Play（Editor → Playing）**：
1. `capture_snapshot()` 保存当前场景
2. 设置 `EditorMode::Play`
3. `AppRuntime::set_paused(false)`
4. 可选：锁定 hierarchy / inspector 的编辑操作

**Pause（Playing → Paused）**：
1. `AppRuntime::set_paused(true)`
2. 设置 `EditorMode::Pause`
3. 解锁 inspector 查看（只读）

**Stop（Playing/Paused → Editor）**：
1. `AppRuntime::set_paused(true)`
2. `restore_snapshot()` 恢复场景状态
3. 设置 `EditorMode::Edit`
4. 清除快照

**Resume（Paused → Playing）**：
1. 设置 `EditorMode::Play`
2. `AppRuntime::set_paused(false)`

### B.3 面板行为随模式变化

| 面板 | Editor Mode | Play Mode | Pause Mode |
|------|-------------|-----------|------------|
| Hierarchy | 可编辑（增删改 GameObject） | 只读 | 只读 |
| Inspector | 可编辑（修改 component） | 只读 | 只读查看运行时值 |
| Scene View | 编辑相机 + gizmo | 游戏相机 / 自由相机 | 编辑相机 + 只读 gizmo |
| Toolbar | ▶ 可用，⏸⏹ 禁用 | ⏸⏹ 可用，▶ 高亮 | ▶⏹ 可用，⏸ 高亮 |
| Stats | 显示 "Edit" | 显示 "Playing" | 显示 "Paused" |

### B.4 EditorContext 扩展

```cpp
class EditorContext {
    // ... existing ...
    EditorMode m_mode = EditorMode::Edit;
    std::optional<SceneSnapshot> m_snapshot;

    void enter_play_mode();
    void enter_pause_mode();
    void stop_to_edit_mode();
    void resume_play_mode();

    bool is_scene_editable() const { return m_mode == EditorMode::Edit; }
};
```

### Checklist — Phase B

- [ ] 实现 `SceneSnapshot` 的 capture / restore（transform + physics state）
- [ ] 在 `EditorContext` 中管理模式切换逻辑和快照生命周期
- [ ] Play 时自动 capture，Stop 时自动 restore
- [ ] Hierarchy / Inspector 根据模式切换只读 / 可编辑
- [ ] Scene View 中 gizmo 在非 Edit 模式下禁用拖拽
- [ ] 处理 Play 中创建 / 删除 GameObject 的恢复（Stop 时丢弃运行时变化）
- [ ] 更新 quickstart 示例演示完整 Play → Pause → Stop 流程

---

## Phase C — 逐帧调试与高级功能

### 目标

增加逐帧推进（Step）功能和辅助调试工具，完善 Game Mode 的调试体验。

### C.1 Step 逐帧推进

**Toolbar 新增按钮**：`[⏭ Step]`（仅在 Pause 模式可用）

行为：
1. 调用 `AppRuntime::step_single_frame()`
2. 执行一个 fixed tick + frame tick + render
3. 保持在 Pause 模式

快捷键：`F6` 或 `Ctrl+Shift+Right`

### C.2 时间缩放

在 Toolbar 中增加时间缩放滑块：

```
[▶]  [⏸]  [⏹]  [⏭]  │  Speed: [0.1x ──●── 2.0x]  │  FPS: 60
```

通过设置 `AppRuntime::set_time_scale(float)` 实现：

```cpp
// app_runtime.hpp
float m_time_scale = 1.0f;

// 在 main loop 中
frame_delta *= m_time_scale;
```

### C.3 Game View 独立面板

**新文件**：`src/rtr/editor/panel/game_view_panel.hpp`

与 Scene View 的区别：

| | Scene View | Game View |
|--|-----------|-----------|
| 相机 | 编辑器自由相机 | 游戏内 Camera 组件 |
| Gizmo | 有 | 无 |
| 用途 | 编辑 | 预览玩家视角 |

Game View 渲染游戏内第一个 active Camera 的视角，不叠加编辑器 overlay。

### C.4 运行时 Inspector 增强

在 Game Mode / Pause 下，Inspector 额外显示：

- Physics debug：速度向量、受力、碰撞接触点
- Component 运行时值实时刷新（而非只显示初始值）
- 在 Pause 模式下允许临时修改值（"hot tweak"，Stop 时丢弃）

### Checklist — Phase C

- [ ] 实现 `AppRuntime::step_single_frame()`
- [ ] Toolbar 增加 Step 按钮，绑定快捷键
- [ ] 实现时间缩放（`m_time_scale`），Toolbar 增加滑块
- [ ] 实现 `GameViewPanel`（独立于 Scene View 的游戏相机视角）
- [ ] Inspector 在 Pause 模式显示运行时 physics debug 数据
- [ ] Pause 模式下的 "hot tweak" — 临时修改值，Stop 时随快照一起丢弃

---

## 与 Headless Plan 的依赖关系

本 plan 与 `headless_plan.md` 存在一个关键交叉点：**两者都需要改动 `AppRuntime`**。

### 改动面对比

| | Editor Mode Plan | Headless Plan |
|--|--|--|
| 改动层级 | 编辑器层（`editor/`） | 运行时 + 渲染层（`app/` + `render/`） |
| 对 AppRuntime | `step_single_frame()` + `m_time_scale` | 拆主循环为 `FrameTimePolicy` + `FrameStepper` + `FrameExecutionPlan` |
| 核心产出 | EditorMode、ToolbarPanel、SceneSnapshot | OfflineRuntime、RenderOutputBackend、FrameContext 解耦 |

### 交叉点

Headless Phase A 会将 `AppRuntime` 的 monolithic 主循环重构为：

- `FrameTimePolicy` — 决定本帧推进多少模拟时间
- `FrameStepper` — 执行 fixed_tick / physics / tick / late_tick
- `FrameExecutionPlan` — 连接两者的数据结构

而本 plan Phase C 的 `step_single_frame()` 和 `m_time_scale` 也要改 `AppRuntime`。

### 建议实施顺序

```
Headless Phase A（抽取 FrameAdvance，重构主循环）
    │
    ├─► Editor Mode Phase A/B（纯 UI + 快照，可随时开始，不涉及 AppRuntime）
    │
    └─► Editor Mode Phase C（step_single_frame / time_scale）
            │
            ├── step_single_frame() 基于 FrameStepper 实现：
            │   stepper.execute(single_step_plan, ctx)
            │
            └── m_time_scale 放入 RealtimeFrameTimePolicy，语义更干净
```

**原因**：

1. 如果先做 Editor Mode Phase C 再做 Headless，`step_single_frame()` 会写在 monolithic 主循环里，Headless 重构时又要迁移
2. 先拆干净主循环，`step_single_frame` 和 `time_scale` 在新结构上实现只需一两行
3. Editor Mode Phase A（ToolbarPanel、EditorMode 枚举、快捷键）和 Phase B（SceneSnapshot）的改动全在编辑器层，与 Headless 完全正交，可以随时并行开发

### 可并行的部分

| 任务 | 是否依赖 Headless Phase A |
|------|---------------------------|
| `EditorMode` 枚举 + `EditorFrameData` 更新 | 否 |
| `ToolbarPanel` UI | 否 |
| `EditorServices::on_mode_change` 回调 | 否 |
| `SceneSnapshot` capture / restore | 否 |
| 面板只读 / 可编辑切换 | 否 |
| `step_single_frame()` | **是** — 需要 `FrameStepper` |
| `m_time_scale` | **是** — 需要 `RealtimeFrameTimePolicy` |
| `GameViewPanel` | 否 |

---

## 实施优先级与依赖

```
Phase A (Toolbar + UI)
    │
    ├── 不依赖任何新基础设施，可与 Headless Plan 并行
    ├── 可立即开始
    └── 完成后即可 Play / Pause 控制模拟
         │
Phase B (Snapshot + Mode)
    │
    ├── 依赖 Phase A 的 EditorMode 枚举
    ├── 依赖 World 的组件遍历能力（已有）
    ├── 可与 Headless Plan 并行
    └── 完成后具备完整的 Edit ↔ Game 工作流
         │
Phase C (Step + Debug)
    │
    ├── 依赖 Phase B 的 Pause 模式
    ├── 依赖 Headless Phase A 的 FrameStepper / FrameTimePolicy
    ├── Game View 可与 Phase B 并行开发
    └── 完成后达到商业引擎级调试体验
```

---

## 风险与注意事项

1. **快照恢复不完整**：v0 只快照 transform + physics state，自定义 component 的运行时状态不会恢复。v1 需要 World 具备通用序列化能力。

2. **Game Mode 下的资源生命周期**：如果 Game Mode 中加载了新资源（texture / mesh），Stop 恢复时这些资源需要被正确释放，否则会内存泄漏。

3. **多 Scene 支持**：当前快照假设单 Scene。如果未来支持多 Scene 同时加载，快照逻辑需要相应扩展。

4. **Input 冲突**：Game Mode 下游戏逻辑可能监听 Escape 键，而编辑器也用 Escape 触发 Stop。需要定义优先级或使用不同的键位。

5. **与 Headless 路径的关系**：`headless_plan.md` 中的 `OfflineRuntime` 不需要 Editor Mode 概念，但 `step_single_frame()` 和时间缩放基于 Headless Phase A 抽出的 `FrameStepper` 实现，接口天然复用。
