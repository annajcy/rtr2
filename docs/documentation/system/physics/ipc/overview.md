# IPC Overview

`src/rtr/system/physics/ipc/` is the data-model layer for the engine's future deformable-body pipeline. It is intentionally separated from the runtime-connected rigid-body path in `src/rtr/system/physics/rigid_body/`.

The current IPC tree is organized by responsibility:

- `core/`: global solver-facing state containers
- `model/`: body metadata, tet geometry, and tet/mesh conversion helpers
- `energy/`: placeholder for elastic, inertial, gravity, and contact energy terms
- `solver/`: placeholder for Newton, line search, and future system assembly

The current implementation focuses on the data that must exist before any FEM or IPC solve can happen:

- a global `3N` state vector (`IPCState`)
- body-to-global mapping metadata (`IPCBodyInfo`)
- tetrahedral rest geometry and precompute (`TetGeometry`, `TetBody`)
- tet-to-render-mesh write-back (`tet_mesh_convert.hpp`)

What is not implemented yet:

- elastic energy evaluation
- global system assembly
- Newton or line-search solve loops
- collision barrier and CCD terms
- runtime integration into `PhysicsSystem::step()`

## Module Relationships

The intended data flow is:

```text
TetGeometry / TetBody
    -> precompute rest-shape data
    -> assemble global DOFs into IPCState
    -> future energy / solver pipeline
    -> tet_to_mesh(...) / update_mesh_positions(...)
    -> ObjMeshData for rendering
```

`model/` owns geometry and mapping metadata. `core/` owns solver-facing global vectors. Future `energy/` and `solver/` code will consume both.

## Current Boundary

The current `ipc/` subtree is deliberately narrow:

- supported: storing nodal DOFs, tet rest geometry, per-vertex lumped mass, and tet surface export
- helper-only: extracting mesh positions and triangles for a future external tetrahedralizer
- not supported: `ObjMeshData -> TetGeometry`, volumetric meshing, or a runtime deformable solve loop

See the per-file pages under this subtree for the detailed data layouts and algorithms.
