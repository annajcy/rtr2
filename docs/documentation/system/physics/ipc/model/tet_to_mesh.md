# `tet_to_mesh.hpp`

`src/rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp` contains the tet-surface extraction and render-mesh write-back path.

## Current Responsibility

This file owns the `TetGeometry / TetBody / IPCState::x -> ObjMeshData` direction.

Public APIs:

- `TetSurfaceMapping`
- `build_tet_surface_mapping(...)`
- `tet_dofs_to_surface_mesh(...)`
- `tet_rest_to_surface_mesh(...)`
- `update_surface_mesh_from_tet_dofs(...)`

This is the runtime-facing bridge used by `IPCTetComponent` and `sync_ipc_to_scene(...)`.

## `TetSurfaceMapping`

Surface extraction follows the same topological rule as before:

$$
\text{surface face} \iff \text{the triangle appears exactly once}
$$

Each tet contributes four faces. Faces that appear once are boundary faces.

The extracted mapping stores:

- `surface_indices`: boundary triangle indices
- `surface_vertex_ids`: unique tet vertex ids used by boundary faces

`surface_vertex_ids` remain body-local ids into the tet mesh.

## Surface Mesh Construction

There are two ways to build renderable surface data:

- `tet_rest_to_surface_mesh(...)` reads positions from `TetGeometry::rest_positions`
- `tet_dofs_to_surface_mesh(...)` reads positions from a `3N` DOF vector

Both paths:

- compact the surface vertex set
- rewrite triangle indices into surface-local indexing
- recompute vertex normals

When the DOF vector is a global multi-body state, `vertex_offset` shifts body-local surface ids into the correct global slice.

## In-Place Write-Back

`update_surface_mesh_from_tet_dofs(...)` reuses an already-built `ObjMeshData` and a cached `TetSurfaceMapping`:

1. update mapped surface vertex positions from the current DOF vector
2. recompute normals

That avoids repeating boundary extraction every frame.

This is the path used by the scene bridge:

```text
TetBody
  -> build_tet_surface_mapping(...)   (once)
  -> tet_rest_to_surface_mesh(...)    (once)
  -> update_surface_mesh_from_tet_dofs(...)  (every sync)
```

## Current Limits

This layer does not:

- preserve a separate high-resolution render mesh
- embed an arbitrary display mesh into the tet mesh
- infer tet data from a surface mesh

It assumes the render surface is the tet boundary surface extracted from the current tet topology.
