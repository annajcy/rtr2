# Material Model 总览

`energy/material_model/` 目录用于放 tet 单元局部的本构模型。

当前文件包括：

- `src/rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/tet_material_variant.hpp`

这一层只关心单个 tet、单个形变梯度 `F` 上的材料响应。现在本构模型和它的物理参数都归这一层自己管理，不直接处理全局 DOF 装配；装配桥接在 [`material_energy.md`](../material_energy.md) 里。
