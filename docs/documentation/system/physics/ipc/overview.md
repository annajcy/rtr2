# IPC Overview

`src/rtr/system/physics/ipc/` is the data-model layer for the engine's future deformable-body pipeline. It is intentionally separated from the runtime-connected rigid-body path in `src/rtr/system/physics/rigid_body/`.

The current IPC tree is organized by responsibility:

- `core/`: global solver-facing state containers
- `model/`: body metadata, tet geometry, and tet/mesh conversion helpers
- `geometry/`: local primitive distance kernels and dense local derivatives
- `energy/`: placeholder for elastic, inertial, gravity, and contact energy terms
- `solver/`: placeholder for Newton, line search, and future system assembly

The current implementation focuses on the data that must exist before any FEM or IPC solve can happen:

- a global `3N` state vector (`IPCState`)
- body-to-global mapping metadata (`IPCBodyInfo`)
- tetrahedral rest geometry and precompute (`TetGeometry`, `TetBody`)
- local PP / PE / PT / EE distance kernels with dense local gradient and Hessian
- surface-to-tet meshing (`mesh_tet_converter/mesh_to_tet.hpp`)
- tet-to-render-mesh write-back (`mesh_tet_converter/tet_to_mesh.hpp`)

What is not implemented yet:

- contact barrier and contact candidate management
- CCD built on top of local distance kernels
- full global contact assembly
- runtime integration through `step_scene_physics(...)` and `ipc_system.step(dt)`

## Module Relationships

The intended data flow is:

```text
TetGeometry / TetBody
    -> precompute rest-shape data
    -> assemble global DOFs into IPCState
    -> future energy / solver pipeline
    -> tet_rest_to_surface_mesh(...) / update_surface_mesh_from_tet_dofs(...)
    -> ObjMeshData for rendering

local primitive coordinates
    -> geometry distance kernels
    -> local dense distance / gradient / hessian
    -> future barrier / CCD / derivative tests
```

`model/` owns body geometry and mapping metadata. `core/` owns solver-facing global vectors. `geometry/` owns local primitive evaluations. Future `energy/`, contact, and solver code will consume those lower layers.

## Current Boundary

The current `ipc/` subtree is deliberately narrow:

- supported: storing nodal DOFs, tet rest geometry, per-vertex lumped mass, tet surface export, and `ObjMeshData -> TetGeometry` / `TetBody` when `RTR_HAS_FTETWILD` is enabled
- not supported: topology repair, cached remeshing, or preserving a separate display mesh from the tet boundary

See the per-file pages under this subtree for the detailed data layouts and algorithms.
