# `mesh_to_tet.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp` contains the surface-mesh-to-tet entry points.

## Current Responsibility

This file owns the `ObjMeshData -> TetGeometry / TetBody` direction.

Public APIs:

- `TetMeshingParams`
- `TetMeshingResult`
- `ftetwild_available()`
- `tetrahedralize_obj_mesh(...)`
- `obj_mesh_to_tet_geometry(...)`
- `obj_mesh_to_tet_body(...)`
- `obj_mesh_to_eigen_positions(...)`
- `obj_mesh_to_triangle_indices(...)`

This header stays header-only inside `src/rtr/`. The actual tetrahedralizer is external and currently gated by `RTR_HAS_FTETWILD`.

## What It Does

The conversion path is:

```text
ObjMeshData
  -> obj_mesh_to_eigen_positions()
  -> obj_mesh_to_triangle_indices()
  -> GEO::Mesh
  -> fTetWild tetrahedralization
  -> TetGeometry
  -> optional TetBody construction
```

The helper functions validate that:

- vertices are non-empty
- indices are non-empty
- indices form triangles
- triangle indices stay in range

## fTetWild Boundary

When `RTR_HAS_FTETWILD=0`:

- `ftetwild_available()` returns `false`
- `tetrahedralize_obj_mesh(...)` returns `success=false`
- convenience wrappers throw with an explanatory message

When `RTR_HAS_FTETWILD=1`, the header:

- builds a `GEO::Mesh` surface
- forwards `TetMeshingParams` into `floatTetWild::Parameters`
- converts fTetWild output matrices into `TetGeometry`

The implementation also normalizes tet orientation. Degenerate or out-of-range output is rejected.

## Result Types

`TetMeshingResult` is the non-throwing boundary:

- `success=true` and `geometry` filled on success
- `success=false` and `error_message` filled on failure

`obj_mesh_to_tet_geometry(...)` and `obj_mesh_to_tet_body(...)` are convenience wrappers for call sites that prefer exceptions.

`obj_mesh_to_tet_body(...)` returns a body before `fixed_vertices` are authored, which keeps it compatible with the existing IPC setup flow:

```text
ObjMeshData
  -> obj_mesh_to_tet_body(...)
  -> mark fixed_vertices / choose material
  -> add IPCTetComponent
```

## Current Limits

This layer does not attempt to:

- repair arbitrary triangle meshes
- preserve the original render-mesh topology
- cache tetrahedralization results
- hide the cost of runtime remeshing

It is an adapter around external tetrahedralization, not a full geometry-processing framework.
