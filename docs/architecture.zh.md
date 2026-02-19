# RTR2 架构概览

本文档介绍引擎的内部结构与模块组织，与 `src/rtr` 目录布局保持一致。

-------

## 编辑器（`src/rtr/editor`）

基于 ImGui 的工具集，用于场景检查和引擎监控。

- **层级面板（Hierarchy Panel）：** 以树状结构展示 World/Scene 层级。
- **检查器面板（Inspector Panel）：** 对 GameObject 和 Component 进行属性编辑。
- **统计与日志（Stats & Logger）：** 性能监控和实时日志查看。

### 编辑器渲染（`src/rtr/editor/render`）

专用渲染 Pass，处理编辑器特有的叠层显示（Gizmo、网格等）。

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

不同引擎模块与外部库之间的桥接逻辑。

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

- **Renderer：** 高层级 Vulkan 渲染器控制。
- **Frame Scheduler：** 管理多缓冲渲染与 Swapchain 交互。
- **Pipelines：** `IRenderPipeline` 的定义及其具体实现。

### 输入系统（`src/rtr/system/input`）

处理来自键盘、鼠标和手柄的用户输入。

- **Input State：** 跟踪所有输入设备的当前状态。
- **Input Events：** 将输入变化传播到引擎其他部分。

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
