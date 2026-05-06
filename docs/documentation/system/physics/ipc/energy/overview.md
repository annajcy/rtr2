# IPC Energy Overview

The `energy/` directory is reserved for the future scalar objective terms used by the deformable solver.

Current files:

- `src/rtr/system/physics/ipc/energy/energy_concept.hpp`
- `src/rtr/system/physics/ipc/energy/inertial_energy.hpp`
- `src/rtr/system/physics/ipc/energy/gravity_energy.hpp`
- `src/rtr/system/physics/ipc/energy/material_model/`
- `src/rtr/system/physics/ipc/energy/material_energy.hpp`

Expected responsibilities include:

- inertial energy
- elastic energy per tet
- gravity
- contact barrier terms
- future gradient and Hessian helpers that are tightly coupled to one energy family

The directory currently contains the first two simple energy terms:

- `Energy`: compile-time interface check for global IPC energy modules
- `InertialEnergy`: the quadratic backward-Euler inertial term
- `GravityEnergy`: the linear gravitational potential term
- `material_model/`: tet material concepts and concrete constitutive models
- `MaterialEnergy`: the bridge from tet-local material response back to global DOFs

Later files in this directory will add tet elastic materials and global assembly support.
