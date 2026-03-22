# Material Model Overview

The `energy/material_model/` directory groups tet-local constitutive behavior.

Current files:

- `src/rtr/system/physics/ipc/energy/material_model/tet_material_model_concept.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/tet_fixed_corotated.hpp`

This layer stays local to one tet and one deformation gradient `F`. It does not know about global DOF assembly; that bridge lives in [`material_energy.md`](../material_energy.md).
