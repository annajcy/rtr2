# `collision_mesh.hpp`

`src/rtr/system/physics/ipc/contact/collision_mesh.hpp` defines the unified surface representation used by IPC contact code.

Its job is to turn heterogeneous scene data into one consistent primitive index space:

- deformable tet boundaries come from `TetSurfaceMapping`
- obstacle triangles come from `ObstacleBody`
- both end up as `CollisionVertex`, `CollisionTriangle`, and `CollisionEdge`

## Why This Layer Exists

The future barrier and CCD code need more than just positions:

- primitive ownership by body
- a stable local surface vertex index
- a way to read current coordinates
- a way to recover global DOF indices for deformable vertices

Those concerns do not belong in `geometry/`, because the geometry kernels are intentionally topology-free. They also do not belong in `IPCState`, because static obstacle vertices are not part of the optimizer unknowns.

`CollisionMesh` is therefore a bridge object, not a solver state container.

## Data Model

The file defines three primitive records plus the aggregate mesh:

```cpp
enum class CollisionVertexKind {
    Deformable,
    Obstacle,
};

struct CollisionVertex {
    IPCBodyID body_id;
    CollisionVertexKind kind;
    std::size_t body_vertex_index;
    std::optional<std::size_t> global_vertex_index;
    Eigen::Vector3d static_position;
};
```

Key points:

- `body_id` tracks primitive ownership
- `body_vertex_index` remembers the local vertex id inside the source body
- `global_vertex_index` is present only for deformable vertices
- `static_position` is used only for obstacle vertices

The aggregate `CollisionMesh` stores:

- `vertices`
- `triangles`
- `edges`
- pre-classified deformable / obstacle index lists for each primitive type

Those cached index lists matter because the broad phase is organized by primitive category, not by rescanning body types every time.

## Position Reading Contract

`read_collision_vertex_position(...)` is the single rule for retrieving a surface vertex position.

For a deformable vertex:

$$
x_i = \texttt{state.position(global\_vertex\_index)}
$$

For an obstacle vertex:

$$
x_i = \texttt{static\_position}
$$

This is the crucial contract that keeps later barrier / CCD wrappers simple. They do not need to know whether a vertex comes from the optimizer state or from a static mesh.

## Mesh Construction

The main entry point is:

```cpp
CollisionMesh build_collision_mesh(const IPCSystem& system);
```

The build happens in three stages.

### 1. Append Tet Surface Vertices and Triangles

For each enabled tet body:

1. call `build_tet_surface_mapping(body)`
2. allocate one `CollisionVertex` for each surface vertex id
3. compute its global vertex index as

$$
\texttt{body.info.dof\_offset / 3 + local\_vertex\_id}
$$

4. remap `surface_indices` into `CollisionTriangle` records in collision-mesh-local indexing

The implementation uses a local map from tet-local surface vertex id to collision-mesh vertex id so triangles can be appended without leaking the original indexing scheme upward.

### 2. Append Obstacle Vertices and Triangles

For each enabled obstacle body:

1. append one obstacle `CollisionVertex` per `positions[i]`
2. store `global_vertex_index = nullopt`
3. copy `static_position = positions[i]`
4. append obstacle triangles with a surface-vertex offset

Obstacle geometry is kept in world space because Day 2 treats it as static.

### 3. Extract and Deduplicate Surface Edges

Edges are not stored directly in tet surface mappings, so the file extracts them from triangles:

- `(v0, v1)`
- `(v1, v2)`
- `(v0, v2)`

Each edge key is stored as:

```cpp
(body_id, min(a, b), max(a, b))
```

Sorting and uniquing that list gives a deterministic edge set.

Including `body_id` in the key is deliberate. It preserves ownership and avoids hiding body boundaries behind a global vertex numbering accident.

## Primitive Classification

After vertices, triangles, and edges are built, `classify_primitives(...)` fills:

- `deformable_vertex_indices`
- `obstacle_vertex_indices`
- `deformable_triangle_indices`
- `obstacle_triangle_indices`
- `deformable_edge_indices`
- `obstacle_edge_indices`

Classification is inferred from the first vertex of each triangle or edge. In the current design each primitive is guaranteed to belong to exactly one body and one side, so this is safe and cheap.

## System Role

`CollisionMesh` is the topology-and-mapping input for the next IPC layers:

- broad phase consumes its primitive lists
- barrier wrappers will read PT / EE positions from it
- CCD wrappers will reuse the same topology while changing trial coordinates

The important design choice is that `CollisionMesh` does not cache deformable positions. That keeps it independent of the current Newton iterate and reusable across trial states.
