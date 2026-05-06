# `tet_body.hpp`

`src/rtr/system/physics/ipc/model/tet_body.hpp` defines the tetrahedral volume model used by the IPC subtree.

## Two Layers: `TetGeometry` and `TetBody`

The file intentionally splits one tet object into:

- `TetGeometry`: rest geometry and per-tet rest-shape precompute
- `TetBody`: physics-facing wrapper with body metadata, material parameters, boundary data, and per-vertex mass

This separation keeps pure geometry tasks independent from body-level physics attributes.

`TetGeometry` owns:

- `rest_positions`
- `tets`
- `Dm_inv`
- `rest_volumes`

`TetBody` adds:

- `info`
- `vertex_masses`
- `material`
- `fixed_vertices`

## Rest-Shape Precompute

For one tet `e = (v_0, v_1, v_2, v_3)`, the rest edge matrix is

$$
D_m = [X_1 - X_0,\; X_2 - X_0,\; X_3 - X_0]
$$

and the deformed edge matrix is

$$
D_s = [x_1 - x_0,\; x_2 - x_0,\; x_3 - x_0]
$$

The deformation gradient for a linear tet is therefore

$$
F D_m = D_s
\quad \Longrightarrow \quad
F = D_s D_m^{-1}
$$

Because `D_m` depends only on rest geometry, `TetGeometry::precompute_rest_data()` computes and caches `D_m^{-1}` once.

## Rest Volume

The same `D_m` also gives the rest volume:

$$
V_e = \frac{|\det(D_m)|}{6}
$$

`|\det(D_m)|` is the volume of the parallelepiped spanned by the three tet edges, so the tetrahedron volume is one sixth of that.

If `\det(D_m)` is numerically zero, the tet is degenerate and precompute rejects it.

## Lumped Vertex Mass

`TetBody::precompute()` turns per-tet volume into lumped nodal mass:

$$
m_a = \sum_{e \ni a} \frac{\rho V_e}{4}
$$

Each tet contributes total mass `\rho V_e`, then distributes it equally to its four vertices. The result is stored in `vertex_masses` and later meant to be written into `IPCState::mass_diag`.

## Why `material` and `fixed_vertices` Live in `TetBody`

`material` is a body-level constitutive choice together with its own physical parameters. The same rest mesh could be simulated with different material laws and different parameter sets, so the whole material object belongs in `TetBody`.

`fixed_vertices` is a body-level boundary-condition mask, not a geometric property. It answers whether a vertex is currently pinned by a Dirichlet condition, so it belongs next to material and mass data rather than inside `TetGeometry`.

## Block Generator

The file also provides:

- `generate_tet_geometry_block(...)`
- `generate_tet_block(...)`

They construct a regular Cartesian block and split each cube cell into six tets.

### Step 1: generate lattice vertices

For input cell counts `(n_x, n_y, n_z)`, the regular lattice contains

$$
(n_x + 1)(n_y + 1)(n_z + 1)
$$

vertices. Each lattice coordinate `(i, j, k)` is mapped into a single `rest_positions` index.

### Step 2: split each cube into 6 tets

Each cube contributes six tetrahedra. The implementation uses the body diagonal `(v_{000}, v_{111})` as the shared spine and fans six tets around it. This gives a deterministic and easy-to-test regular tetization pattern.

### Step 3: fix local orientation

After emitting each tet, the generator checks the sign of `\det(D_m)`. If the orientation is negative, it swaps two local vertices so the tet has positive rest orientation before later precompute.

## Why This File Matters

`tet_body.hpp` is the point where the continuous FEM story first becomes concrete engine data:

- a volume is represented by shared tet vertices and connectivity
- rest-shape quantities are precomputed once
- body-level mass and boundary metadata are attached without polluting geometry-only code
