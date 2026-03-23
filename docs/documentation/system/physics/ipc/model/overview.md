# IPC Model Overview

The `model/` directory contains the IPC-side geometry and body metadata layer.

Current files:

- `src/rtr/system/physics/ipc/model/ipc_body.hpp`: body categories and body-to-global mapping metadata
- `src/rtr/system/physics/ipc/model/tet_body.hpp`: tetrahedral geometry, rest-shape precompute, body-level material data, and block generators
- `src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp`: `ObjMeshData` to `TetGeometry` / `TetBody`
- `src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp`: tet-surface extraction and tet-to-render-mesh conversion
- `src/rtr/system/physics/ipc/model/obstacle_body.hpp`: placeholder obstacle body type

This layer answers:

- what a deformable body looks like in rest configuration
- how one body maps into the global `IPCState`
- how surface meshes are tetrahedralized
- how tet geometry is turned into a renderable surface mesh

It does not yet answer how energies, gradients, Hessians, or line searches are computed. Those belong to the future `energy/` and `solver/` layers.
