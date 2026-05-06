# `ipc_body.hpp`

`src/rtr/system/physics/ipc/model/ipc_body.hpp` defines the minimal metadata shared by IPC-side bodies.

## `IPCBodyType`

`IPCBodyType` distinguishes which kind of body the solver is dealing with:

- `Tet`
- `Shell`
- `Obstacle`

Only `Tet` and the `ObstacleBody` placeholder exist in the current tree, but the enum is already structured for future mixed-body systems.

## `IPCBodyInfo`

`IPCBodyInfo` is the bridge between one body and the future global solve vectors:

- `type`: concrete IPC body category
- `dof_offset`: first global DOF index in `IPCState`
- `vertex_count`: number of vertices owned by the body
- `enabled`: whether the body currently participates in assembly and solve

The key field is `dof_offset`. Since the global IPC state is vertex-major and each vertex contributes three translational DOFs, `dof_offset` points at the first entry of this body's slice inside the global vectors.

If a body starts at vertex offset `k`, then:

$$
\text{dof\_offset} = 3k
$$

and its position slice in the global state has length `3 * vertex_count`.

## Why This Metadata Stays Minimal

`IPCBodyInfo` intentionally does not duplicate geometry, material, or mass data. Its purpose is narrower:

- identify the body category
- describe where the body's local vertices live in the global vectors
- allow temporary enable/disable during assembly

That keeps body-local model data in `TetBody`, while still giving the future solver a compact way to map local data into global storage.
