# `tet_mesh_convert.hpp`

`src/rtr/system/physics/ipc/model/tet_mesh_convert.hpp` contains the current tet/mesh conversion layer.

## Current Boundary

The file has two roles:

- production path: `TetGeometry` / `TetBody` / `IPCState::x` to renderable `ObjMeshData`
- helper-only path: `ObjMeshData` to typed position and triangle arrays for a future external tetrahedralizer

What it does not do:

- surface-mesh tetrahedralization
- `ObjMeshData -> TetGeometry`
- topology repair, watertightness repair, or orientation repair for arbitrary triangle meshes

The future intended path is:

```text
ObjMeshData
  -> mesh_positions_to_eigen()
  -> mesh_triangles()
  -> external tetrahedralizer
  -> TetGeometry
```

## `extract_tet_surface(...)`

Surface extraction is based on a simple topological rule:

$$
\text{surface face} \iff \text{the triangle appears exactly once}
$$

Each tet contributes four triangular faces. The implementation sorts the three vertex ids of each face to build a counting key, while also keeping the oriented local order for output.

The result is stored in `TetSurfaceResult`:

- `surface_indices`: boundary triangle indices, still referencing original tet vertex ids
- `surface_vertex_ids`: unique tet vertex ids touched by boundary faces

This split is useful because the render surface only uses a subset of the full tet vertex set.

## `tet_to_mesh(...)`

`tet_to_mesh(...)` turns `TetSurfaceResult` into a compact `ObjMeshData`.

### Step 1: build a compact surface vertex map

The function builds a map

$$
\text{old tet vertex id} \rightarrow \text{new surface vertex id}
$$

using `surface_vertex_ids`. This removes interior vertices from the render mesh.

### Step 2: read positions

There are two entry points:

- `tet_to_mesh(const Eigen::VectorXd& positions, ..., vertex_offset)`
- `tet_to_mesh(const TetGeometry& geometry, ...)`

The first reads vertex positions from a global `3N` DOF vector:

$$
x_i =
\begin{bmatrix}
\mathbf{x}_{3i} \\
\mathbf{x}_{3i+1} \\
\mathbf{x}_{3i+2}
\end{bmatrix}
$$

When `positions` is the full `IPCState::x`, the optional `vertex_offset` is used to shift body-local surface vertex ids into global vertex ids:

$$
\text{global vertex id} = \text{vertex offset} + \text{body-local vertex id}
$$

This is what the scene bridge uses for multi-body write-back.

The second entry point reads positions directly from `geometry.rest_positions`.

### Step 3: rewrite triangle indices

`surface_indices` still references tet-global vertex ids, so the function rewrites them into compact 0-based surface-mesh indices for `ObjMeshData::indices`.

## Normal Recompute

After positions and triangle indices are assembled, normals are recomputed from geometry. For one triangle `(i_0, i_1, i_2)`, the face normal is

$$
n_f = \mathrm{normalize}\left((p_1 - p_0) \times (p_2 - p_0)\right)
$$

Face normals are accumulated onto adjacent vertices, then normalized per vertex. If a vertex receives a zero-length accumulated normal, the implementation falls back to `(0, 1, 0)`.

## `update_mesh_positions(...)`

Surface extraction and remapping depend only on tet topology, so they can be cached in `TetSurfaceResult`.

`update_mesh_positions(...)` reuses that cached mapping:

1. update already-mapped surface vertex positions from the current DOF vector
2. recompute normals

When the input DOF vector is a global multi-body `IPCState::x`, the function also accepts a `vertex_offset` so body-local surface ids can be mapped into the correct global slice.

That is cheaper than repeating boundary extraction every frame, and it is the path used by `sync_ipc_to_scene(...)`.

## Mesh-to-Tet Helpers

`mesh_positions_to_eigen(...)` converts `ObjMeshData` vertex positions into `std::vector<Eigen::Vector3d>`.

`mesh_triangles(...)` converts the flat `indices` array into `std::vector<std::array<uint32_t, 3>>` and validates that the indices are triangular and in range.

These helpers are intentionally narrow. They prepare input for a future external tetrahedralizer; they do not build `TetGeometry` by themselves.
