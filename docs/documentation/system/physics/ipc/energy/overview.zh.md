# IPC Energy 总览

`energy/` 目录用于放 deformable 求解器里的各类标量目标函数项。

当前文件包括：

- `src/rtr/system/physics/ipc/energy/energy_concept.hpp`
- `src/rtr/system/physics/ipc/energy/inertial_energy.hpp`
- `src/rtr/system/physics/ipc/energy/gravity_energy.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/`
- `src/rtr/system/physics/ipc/energy/material_energy.hpp`

未来这里预计会承载：

- 惯性能
- 每个 tet 的弹性能
- 重力势能
- 接触 barrier 项
- 与某一类能量紧密耦合的梯度和 Hessian 辅助

当前已经实现了两个最基础的能量项：

- `Energy`：全局 IPC energy 模块的编译期接口约束
- `InertialEnergy`：Backward Euler 的二次惯性能
- `GravityEnergy`：线性的重力势能
- `material_model/`：tet 材料 concept 和具体本构模型
- `MaterialEnergy`：把单元级材料响应桥接回全局 DOF 的 energy 层

后续这个目录会继续加入 tet 材料模型和全局装配相关文件。
