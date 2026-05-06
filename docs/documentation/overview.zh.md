# RTR2 架构概览

本文档介绍引擎的内部结构与模块组织，与 `src/rtr` 目录布局保持一致。

-------

## 编辑器（`src/rtr/editor`）

基于 ImGui 的工具集，用于场景检查和引擎监控。

- **层级面板（Hierarchy Panel）：** 以树状结构展示 World/Scene 层级。
- **检查器面板（Inspector Panel）：** 对 GameObject 和 Component 进行属性编辑。
- **统计与日志（Stats & Logger）：** 性能监控和实时日志查看。

### 编辑器渲染（`src/rtr/editor/render`）

负责 editor 专用的输出合成和 UI 渲染。

- **Editor Output Backend：** 消费纯场景 pipeline 的输出，再合成 editor UI。
- **Editor ImGui Pass：** 把 ImGui 和 scene view 内容画到最终输出目标。

-------

## 应用层（`src/rtr/app`）

应用层负责管理引擎的生命周期和主执行循环。

- **主入口（Main Entry）：** `AppRuntime` 协调所有引擎子系统。
- **主循环（Main Loop）：** 处理事件、更新逻辑和帧调度。

## 框架层（`src/rtr/framework`）

核心引擎逻辑与场景图管理。

### 核心（`src/rtr/framework/core`）

定义 RTR2 场景的基础构建块。

- **World：** 所有对象的顶层容器。
- **Scene：** 管理 GameObject 的集合。
- **GameObject：** 所有实体类对象的基类。

### 组件（`src/rtr/framework/component`）

经典面向对象的组件系统，用于扩展 GameObject 功能。

- **Base：** 所有逻辑特性的 `Component` 基类。
- **Camera & Controls：** 相机组件与标准控制器逻辑。
- **Material：** 与渲染资源系统的集成接口。
- **PBPT 集成：** 专用于路径追踪桥接的组件（如区域光、特定 BSDF 数据）。

### 集成（`src/rtr/framework/integration`）

框架层与下游系统或外部库之间的桥接逻辑。

- **系统集成：**
    - 场景到物理系统的同步辅助逻辑。
    - 场景到渲染视图的提取构建器。
- **PBPT 集成：**
    - `pbpt_scene_importer.hpp`：将 PBPT XML 格式的资产加载到引擎中。
    - `pbpt_scene_export_builder.hpp`：将实时场景导出到路径追踪器。
    - `pbpt_offline_render_service.hpp`：触发和监控离线路径追踪渲染的逻辑。

### 资源（`src/rtr/framework/resource`）

场景图内使用的资源（网格、材质）的框架层封装。

## 资源层（`src/rtr/resource`）

资产管理层。

- **ResourceManager：** 纹理、网格和材质的异步加载与缓存。
- **Resource Types：** 资产类型及其元数据定义。

## 系统层（`src/rtr/system`）

负责协调特定引擎功能的高层系统。

### 渲染系统（`src/rtr/system/render`）

管理 Vulkan 渲染管线、帧调度和资源同步。

- **Renderer：** 按 output backend 参数化的 swapchain 渲染引导层。
- **Frame Scheduler：** 负责多缓冲 swapchain acquire / submit / present 调度。
- **Output Backends：** 处理 realtime present、editor 合成和 preview/export 输出。
- **Pipelines：** 只产出最终 offscreen image 的纯内容 pipeline。

### 输入系统（`src/rtr/system/input`）

处理来自键盘、鼠标和手柄的用户输入。

- **Input State：** 跟踪所有输入设备的当前状态。
- **Input Events：** 将输入变化传播到引擎其他部分。

### 物理系统（`src/rtr/system/physics`）

负责当前运行时中的刚体与 IPC 形变模拟。

- **PhysicsSystem：** 同时持有 `rb::RigidBodySystem` 与 `ipc::IPCSystem`。
- **Fixed Tick 集成：** 框架层通过 `step_scene_physics(...)` 把 scene graph、`rb::RigidBodySystem` 和 IPC 运行时连接起来。
- **文档入口：** 详见 `docs/documentation/system/physics/` 下的总览、运行时集成、IPC bridge/example 与 Rigid Body Dynamics。

## RHI（`src/rtr/rhi`）

**渲染硬件接口（Rendering Hardware Interface）**：Vulkan 对象的低层级 RAII 封装。

- **Context & Device：** Vulkan 实例和逻辑设备管理。
- **Buffer & Image：** GPU 内存资源管理。
- **Shader Module：** 编译后的 Slang/SPIR-V 着色器的加载与管理。

## 工具库（`src/rtr/utils`）

通用辅助函数和样板代码简化工具。

- **Logging：** 基于 `spdlog` 的自定义封装。
- **Math Helpers：** `pbpt::math` 与渲染友好格式之间的转换工具。
- **Profiling：** 基于宏的性能分析插桩。
