# 物理系统总览 (Overview)

本文档是对 `rtr2` 中自定义物理引擎 (`rtr::system::physics`) 的架构总览解析。物理引擎主要基于半隐式欧拉（蛙跳法）进行刚体动力学模拟。对于具体的理论实现细节，可以参考刚体动力学文档。

## 架构总览

整个物理模块的架构分为以下几个主要层次和组件：

1. **`PhysicsSystem` (物理与场景的桥梁)**
   - **指责**：该类负责将底层的独立物理世界与上层的框架组件（ECS 结构的 `GameObject`，主要是 `RigidBody` 和 `Collider` 组件）连接起来。
   - **交互流程**：在引擎的固定时间步 (`fixed_tick`) 中，它依次调用 `sync_scene_to_physics()`（将场景中节点的位姿同步给碰撞体和运动学刚体）、`m_physics_world.tick()` 进行模拟步进，最后调用 `sync_physics_to_scene()` 将受物理驱动的动态（Dynamic）刚体的最新位姿同步回场景节点中。
2. **`PhysicsWorld` (核心物理沙盒)**
   - **指责**：负责维护所有的刚体结构 (`RigidBody`) 和碰撞体结构 (`Collider`)，是发生物理模拟本身的核心对象。
   - **核心管线 (`tick`)**：每一帧的时间更新中，顺序执行力与位移积分 (`integrate_forces_and_drift`)、碰撞检测 (`generate_contacts`)、碰撞点求解约束 (`solve_contacts`)，及最终外部可观测速度的更新 (`update_observable_velocities`)。
3. **`RigidBody` (刚体数据容器)**
   - **指责**：纯数据对象，内部包含了刚体的运动状态（`RigidBodyState`），分为平移状态 (`TranslationState`：位置和速度) 和旋转状态 (`RotationalState`：四元数朝向和角速度)，以及力量累加器 (`ForceAccumulator`)。
   - **特殊字段**：为了兼容蛙跳法半隐式积分，该类还维护了半步长速度 (`m_half_step_linear_velocity` 和 `m_half_step_angular_velocity`)。
