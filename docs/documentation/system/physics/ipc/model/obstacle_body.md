# `obstacle_body.hpp`

`src/rtr/system/physics/ipc/model/obstacle_body.hpp` defines the static obstacle representation currently used by the IPC contact pipeline.

## Current Data Shape

`ObstacleBody` stores:

- `IPCBodyInfo info{.type = IPCBodyType::Obstacle}`
- `positions`: world-space obstacle vertices
- `triangles`: triangle topology over `positions`
- `edges`: deduplicated undirected edges extracted from triangles

The obstacle is deliberately outside the global optimizer state:

- obstacle vertices are not appended to `IPCState.x`
- `info.dof_offset` stays unused for solver assembly
- contact code reads obstacle coordinates directly from `positions`

That is the right model for the current Day 2 scope, where obstacles are static geometry used only by contact preprocessing.

## Validation and Edge Extraction

The file provides two behavior helpers:

- `validate()`: checks that every triangle index is inside `positions`
- `build_edges()`: extracts `(i, j)` edges from triangle faces, canonicalizes them with `min/max`, then sorts and uniques them

This makes the type immediately usable by `contact/collision_mesh.hpp`, which needs triangle and edge primitives but should not reimplement obstacle topology cleanup.

## Why Obstacles Live in `model/`

`ObstacleBody` belongs in `model/` because it is geometry and ownership metadata, not solver state:

- `core/` owns global DOF vectors for deformable unknowns
- `model/` owns body-local geometry and topology
- `contact/` later combines `ObstacleBody` with tet boundary data into one collision surface

Keeping the obstacle mesh here prevents static geometry from contaminating the optimizer-facing `IPCState` layout.

## Current Boundary

The current file intentionally stays minimal:

- supported: static triangle meshes, validation, edge extraction
- not yet supported: kinematic obstacle updates, rigid transforms, normals, UVs, or contact caching

That is enough for the first collision-mesh and broad-phase pipeline without locking the future obstacle API too early.
