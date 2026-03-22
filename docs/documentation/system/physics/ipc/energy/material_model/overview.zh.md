# Material Model 总览

`energy/material_model/` 目录用于放 tet 单元局部的本构模型。

当前文件包括：

- `src/rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp`

这一层只关心单个 tet、单个形变梯度 `F` 上的材料响应，不直接处理全局 DOF 装配；装配桥接在 [`material_energy.md`](../material_energy.md) 里。
