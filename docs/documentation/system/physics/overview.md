# Physics System Overview

RTR2 currently has two physics directions under `src/rtr/system/physics/`:

- a runtime-connected rigid-body subsystem in `rigid_body/`
- an in-progress IPC/FEM subsystem in `ipc/`

`PhysicsSystem` currently owns only `RigidBodyWorld`. The IPC directory already contains the first batch of data structures, but it is not wired into the runtime loop yet.

The authoritative detailed write-up is currently the Chinese documentation:

- `overview.zh.md`
- `runtime-integration.zh.md`
- `rigid-body-dynamics.zh.md`

## Current Scope

- Implemented: rigid-body simulation, collision detection/response, scene/physics synchronization, IPC/FEM core data structures.
- Not implemented: cloth runtime, IPC solver loop, IPC contact/barrier/CCD, rigid-body/IPC coupling.

## Fixed-Step Flow

```text
step_scene_physics(scene, physics_system, dt)
    -> sync_scene_to_rigid_body(...)
    -> PhysicsSystem::step(dt)
         -> RigidBodyWorld::step(dt)
    -> sync_rigid_body_to_scene(...)
```

`PhysicsSystem::step()` performs world-local simulation only. Scene synchronization stays in the framework integration layer. The IPC subsystem is not part of this flow yet.

## IPC Global State: `IPCState`

`ipc::IPCState` is the future IPC/FEM solver's global state container. It is not a per-body cache. Instead, it stacks the degrees of freedom of all deformable bodies into one finite-dimensional vector.

The reason is the standard FEM discretization step:

- the continuous system starts from a displacement field `x(X, t)`
- after spatial discretization, that field is represented by nodal values
- in 3D, each vertex contributes 3 translational DOFs
- therefore `N` vertices become `3N` unknowns

`IPCState` stores exactly those unknowns.

Using linear FEM notation, the continuous field is approximated as

$$
\hat{x}(X) = \sum_{a=1}^{N} x_a N_a(X)
$$

where `N_a(X)` is the shape function of vertex `a`, and `x_a \in \mathbb{R}^3` is that vertex position. Once the field is written in terms of nodal values, all unknowns can be stacked into one global vector:

$$
\mathbf{x} =
\begin{bmatrix}
x_1 \\
x_2 \\
\vdots \\
x_N
\end{bmatrix}
\in \mathbb{R}^{3N}
$$

### Fields

- `x`: current configuration, stacked as a global `3N` vector
- `x_prev`: previous-step configuration
- `v`: stacked nodal velocities, also `3N`
- `mass_diag`: lumped diagonal mass stored per DOF, also `3N`

If vertex `i` has scalar mass `m_i`, the diagonal mass matrix is represented as

$$
\mathbf{M} = \mathrm{diag}(m_1, m_1, m_1, m_2, m_2, m_2, \ldots, m_N, m_N, m_N)
$$

and `mass_diag` stores that diagonal directly.

### Layout

The layout is vertex-major:

```text
x = [x0x, x0y, x0z, x1x, x1y, x1z, ..., x(N-1)z]^T
```

Equivalently,

$$
\mathbf{x} =
[x_{0x}, x_{0y}, x_{0z}, x_{1x}, x_{1y}, x_{1z}, \ldots, x_{(N-1)z}]^T
$$

So vertex `i` starts at index `3 * i`, and `position(i)` is implemented as `x.segment<3>(3 * i)`.

### Why this matters

Later, inertial energy, elastic energy, gravity, contact terms, gradients, and Hessians will all be assembled against the same global variable. `IPCBodyInfo::dof_offset` exists precisely to map one body's local vertices into this global `IPCState` layout.

For example, the inertial term in implicit integration is commonly written as

$$
E_I(\mathbf{x}) = \frac{1}{2h^2}(\mathbf{x} - \hat{\mathbf{x}})^T \mathbf{M} (\mathbf{x} - \hat{\mathbf{x}})
$$

With a diagonal lumped mass, this becomes a simple per-DOF sum, which is why `IPCState` keeps `mass_diag` as a vector instead of building a sparse mass matrix.

## IPC Body Metadata

`ipc::IPCBodyInfo` is the minimal metadata bridge between one deformable body and the future global IPC solve state.

- `type`: concrete IPC body category such as `Tet`, `Shell`, or `Obstacle`.
- `dof_offset`: first global DOF index in `IPCState`.
- `vertex_count`: number of vertices owned by the body.
- `enabled`: whether assembly/solve should currently include the body.

Together, `dof_offset` and `vertex_count` define which contiguous slice of the global `IPCState` vectors belongs to that body.

## IPC Tet Mesh Data: `TetGeometry` and `TetBody`

The tet model is intentionally split into two layers:

- `ipc::TetGeometry`: tetrahedral mesh geometry and rest-shape precompute
- `ipc::TetBody`: physics-facing wrapper that adds body metadata, material parameters, boundary conditions, and per-vertex mass

This separation matters because geometry-level tasks such as tet validation, tet/mesh conversion, or future tet-to-triangle conversion should not depend on density, elastic parameters, or pinned vertices.

At a high level:

- `TetGeometry` owns `rest_positions`, `tets`, `Dm_inv`, and `rest_volumes`
- `TetBody` owns `info`, `geometry`, `vertex_masses`, material parameters, and `fixed_vertices`

## Current Tet/Mesh Conversion Boundary

The current `tet_mesh_convert.hpp` is intentionally scoped to the lightweight conversion layer:

- supported: `TetGeometry` / `TetBody` / `IPCState::x` to surface `ObjMeshData`
- supported: `ObjMeshData` to position and triangle helper arrays
- not supported: `ObjMeshData` to volumetric `TetGeometry`

In other words, `tet_to_mesh(...)` and `update_mesh_positions(...)` are production paths for render write-back, while `mesh_positions_to_eigen(...)` and `mesh_triangles(...)` are only preparation helpers for a future external tetrahedralizer.

The intended future path is:

```text
ObjMeshData
  -> mesh_positions_to_eigen()
  -> mesh_triangles()
  -> external tetrahedralizer (future)
  -> TetGeometry
```

That volumetric meshing step is explicitly out of scope for the current implementation.

## Current `tet -> mesh` Algorithm

The current `tet -> mesh` path exists to turn a volumetric tet representation into a renderable surface `ObjMeshData`. It is not volumetric remeshing. The implementation is a combination of:

- surface extraction
- surface-vertex remapping
- normal recomputation

### Step 1: detect boundary triangles

Each tet contributes four triangular faces. A face shared by two tets is internal; a face that appears exactly once is on the boundary:

$$
\text{surface face} \iff \text{face appears exactly once}
$$

So `extract_tet_surface(...)` enumerates the 4 faces of every tet, stores a sorted face key for counting, and preserves the oriented local face for output winding.

The result is:

- `surface_indices`: boundary triangle indices, still referencing original tet vertex ids
- `surface_vertex_ids`: unique tet vertex ids that lie on the boundary

### Step 2: remap tet vertex ids to compact surface vertex ids

The surface only uses a subset of the tet vertices, so the render mesh should not keep all volumetric vertices. The algorithm therefore builds

$$
\text{old tet vertex id} \rightarrow \text{new surface vertex id}
$$

and rewrites `surface_indices` into compact 0-based triangle indices for `ObjMeshData`.

This keeps the render mesh minimal and makes later per-frame updates cheaper.

### Step 3: read positions from either DOFs or rest geometry

There are two `tet_to_mesh(...)` entry points:

- `tet_to_mesh(const Eigen::VectorXd& positions, ...)`
  reads current positions from the global IPC DOF vector
- `tet_to_mesh(const TetGeometry& geometry, ...)`
  reads positions from `rest_positions`

For the DOF-vector version, vertex `i` comes from

$$
x_i =
\begin{bmatrix}
\mathbf{x}_{3i} \\
\mathbf{x}_{3i+1} \\
\mathbf{x}_{3i+2}
\end{bmatrix}
$$

which is why this is the main write-back path from `IPCState::x` to the renderer mesh.

### Step 4: recompute vertex normals

After positions and triangle indices are assembled, the algorithm recomputes per-vertex normals from the surface triangles. For one triangle `(i_0, i_1, i_2)`, the face normal is

$$
n_f = \mathrm{normalize}\left((p_1 - p_0) \times (p_2 - p_0)\right)
$$

and each vertex normal accumulates adjacent face normals before normalization.

This matches the same general strategy used in `obj_io.hpp` when an OBJ file has no input normals.

### Step 5: why `update_mesh_positions(...)` is cheaper

Surface extraction and remapping depend only on tet topology, not on the current deformation. Therefore they can be cached.

Once `TetSurfaceResult` is known, each frame only needs to:

1. update the already-mapped surface vertex positions
2. recompute normals

instead of rebuilding the entire boundary extraction pipeline.

So the current algorithm solves:

- how a volumetric tet mesh becomes a renderable surface mesh
- how current IPC DOFs are written back into renderer-facing data

It does not solve surface-mesh-to-volume tetrahedralization.

## Tet Block Generator: `generate_tet_geometry_block()`

The project also needs a procedural tet-mesh generator so tests and smoke cases do not depend on external volumetric assets. The current implementation is `generate_tet_geometry_block()` with the convenience wrapper `generate_tet_block()`.

### Inputs

The inputs are:

- `nx, ny, nz`: number of cube cells along x, y, and z
- `spacing`: lattice spacing
- `origin`: starting corner of the block

The function first generates a Cartesian lattice, so the vertex count is

$$
(n_x + 1)(n_y + 1)(n_z + 1)
$$

because `n_x` cubes require `n_x + 1` grid layers.

### Step 1: generate lattice vertices

The generated rest-space points are

$$
X_{ijk} = \text{origin} + \text{spacing} \cdot (i, j, k)^T
$$

for

$$
0 \le i \le n_x,\quad 0 \le j \le n_y,\quad 0 \le k \le n_z
$$

and the 3D lattice index is flattened by

$$
\text{vertex\_index}(i,j,k) = i + (n_x + 1)\bigl(j + (n_y + 1)k\bigr)
$$

### Step 2: split each cube into 6 tetrahedra

For each cube, the 8 corners are

```text
v000, v100, v010, v110, v001, v101, v011, v111
```

The implementation uses a fixed 6-tet decomposition:

$$
(v000, v100, v110, v111)
$$

$$
(v000, v100, v101, v111)
$$

$$
(v000, v001, v101, v111)
$$

$$
(v000, v001, v011, v111)
$$

$$
(v000, v010, v110, v111)
$$

$$
(v000, v010, v011, v111)
$$

Geometrically, these six tets fan around the body diagonal

$$
v000 \leftrightarrow v111
$$

which makes the construction simple and regular.

So the total tet count is

$$
6 n_x n_y n_z
$$

### Step 3: fix local orientation

Even if the geometry is correct, the local tet ordering may produce a negative determinant:

$$
D_m =
\begin{bmatrix}
X_1 - X_0 & X_2 - X_0 & X_3 - X_0
\end{bmatrix}
$$

and

$$
\det(D_m)
$$

encodes the local orientation.

Since later FEM precomputation expects a positive-volume convention, the generator checks the determinant of each candidate tet and swaps two local vertices whenever the determinant is negative. This does not change the geometry, only the local vertex ordering.

### Why this generator is useful

`generate_tet_geometry_block()` is not a general tetrahedralizer. Its role is to generate a:

- regular
- healthy
- predictable
- dependency-free

volume mesh for tests and small demos.

In short, the algorithm is:

1. generate a regular 3D lattice
2. split each cube into 6 tetrahedra
3. normalize each tet orientation

Mathematically, a deformable 3D body is approximated by a union of tetrahedra

$$
\Omega^0 \approx \bigcup_{e=1}^{n_{tet}} T_e
$$

where each element `T_e` is a linear tet.

### Why `TetGeometry` precomputes `D_m^{-1}`

For one tet `(v_0, v_1, v_2, v_3)`, define the rest-shape edge matrix

$$
D_m =
\begin{bmatrix}
X_1 - X_0 & X_2 - X_0 & X_3 - X_0
\end{bmatrix}
$$

and the deformed edge matrix

$$
D_s =
\begin{bmatrix}
x_1 - x_0 & x_2 - x_0 & x_3 - x_0
\end{bmatrix}
$$

For linear tets, the deformation gradient is constant inside the element and satisfies

$$
F D_m = D_s
\quad \Longrightarrow \quad
\boxed{F = D_s D_m^{-1}}
$$

Since `D_m` depends only on the rest geometry, `TetGeometry::precompute_rest_data()` caches `D_m^{-1}` once and reuses it during the solve.

### Why rest volume is `|det(D_m)| / 6`

The determinant of `D_m` gives the volume of the parallelepiped spanned by the three rest-shape edge vectors. A tetrahedron occupies one sixth of that volume:

$$
\boxed{V_e = \frac{|\det(D_m)|}{6}}
$$

This is exactly what `TetGeometry::rest_volumes` stores.

### Why vertex mass is accumulated as `\rho V_e / 4`

The consistent mass matrix is

$$
M_{ab} = \int_{\Omega^0} \rho_0 N_a N_b \, dX
$$

but for graphics simulation, a lumped diagonal mass is usually preferred. For a linear tet, the standard simple approximation is to distribute the element mass equally to its four vertices:

$$
m_e = \rho V_e
$$

$$
\boxed{m_a \mathrel{+}= \frac{\rho V_e}{4}}
\quad \text{for each vertex } a \in e
$$

That is why `TetBody::precompute()` builds `vertex_masses` from `TetGeometry::rest_volumes`; a later system layer can then expand them into the global `IPCState::mass_diag`.

### Why the fixed mask lives in `TetBody`

`fixed_vertices` is the current minimal representation of Dirichlet boundary conditions. It marks which vertices are pinned in this body, while the future solver will decide how those vertex constraints are enforced in global DOF space.

## Algorithm Map

| Subsystem | Current algorithm / responsibility | Primary entry points |
| --- | --- | --- |
| Rigid Body | semi-implicit Euler + contact generation + PGS + positional correction | `RigidBodyWorld::step()` |
| IPC Data Layer | global DOF storage, tet rest-state precompute, tet->mesh write-back, mesh helper conversion, body metadata | `ipc/core/`, `ipc/model/` |
| Integration | scene/physics synchronization across fixed ticks | `step_scene_physics()` |
