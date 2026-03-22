# IPC Scene Bridge

This page documents the framework-side bridge between the IPC deformable runtime and scene/render components.

The relevant runtime-facing code lives in:

- `src/rtr/framework/component/physics/ipc/ipc_tet_component.hpp`
- `src/rtr/framework/integration/physics/ipc_scene_sync.hpp`
- `src/rtr/framework/integration/physics/scene_physics_step.hpp`

## Why a Scene Bridge Exists

`IPCSystem` owns the authoritative deformable nodal state in `IPCState::x`.

The renderer, however, does not consume tet-volume state directly. It consumes a surface mesh through `DeformableMeshComponent`, whose underlying resource is a renderable `ObjMeshData`.

That means the runtime needs a bridge layer that:

1. remembers which scene object corresponds to which tet body;
2. remembers how the tet boundary maps onto the render mesh;
3. updates the render-facing mesh after each IPC solve step.

## `IPCTetComponent`

`IPCTetComponent` is the scene-side cache object for one IPC tet body.

It stores:

- `body_index`: which body inside `IPCSystem` this object corresponds to;
- `surface_cache`: the cached `TetSurfaceResult` from one-time boundary extraction;
- `mesh_cache`: a reusable `ObjMeshData` that can be updated in place every frame.

### What It Is Not Responsible For

`IPCTetComponent` does **not** create the render mesh on its own.

The initial render mesh still comes from the body:

```text
TetBody
  -> extract_tet_surface(body)
  -> tet_to_mesh(body.geometry, surface)
  -> initial ObjMeshData
```

Only after that initial mesh exists do we:

```text
initial ObjMeshData
  -> create DeformableMesh resource
  -> add DeformableMeshComponent
  -> add IPCTetComponent(body_index, surface, mesh_cache)
```

This separation matters because it keeps responsibilities clean:

- `TetBody` provides the volumetric source data;
- `DeformableMeshComponent` owns the render-facing mesh handle;
- `IPCTetComponent` owns only bridge metadata and caches.

## Registration Flow

The current minimum registration flow for one IPC tet object is:

```text
1. Build TetBody
2. Mark fixed vertices / choose material
3. extract_tet_surface(body)
4. tet_to_mesh(body.geometry, surface)
5. create DeformableMesh resource from the initial mesh
6. add DeformableMeshComponent to the GameObject
7. add the TetBody to IPCSystem
8. add IPCTetComponent(body_index, surface, mesh_cache)
9. call ipc_system.initialize()
```

`body_index` is the stable bridge key here. It lets the scene object later recover the body's global vertex offset from `ipc_system.tet_body(body_index).info.dof_offset`.

## `sync_ipc_to_scene(...)`

`sync_ipc_to_scene(scene, ipc_system)` is the per-fixed-tick write-back function.

For each active object:

1. find `IPCTetComponent`;
2. find the colocated `DeformableMeshComponent`;
3. look up the referenced tet body in `IPCSystem`;
4. recover that body's global vertex offset from `body.info.dof_offset / 3`;
5. update `mesh_cache` in place from the global DOF vector;
6. extract positions and normals;
7. call `apply_deformed_surface(...)`.

## Why `body_index` Alone Is Not Enough

`TetSurfaceResult.surface_vertex_ids` are **body-local** vertex ids.  
`IPCState::x`, on the other hand, is a **global** concatenated `3N` vector across all bodies.

So the bridge must map:

$$
\text{global vertex id} = \text{body vertex offset} + \text{local surface vertex id}
$$

This is why the current `tet_mesh_convert.hpp` write-back helpers accept a `vertex_offset` parameter for runtime use.

Without that offset, a second or third body would accidentally read the first body's positions from the global state vector.

## `scene_physics_step(...)` Order

The framework-side fixed-step order is now:

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_world());
physics_system.step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_world());

physics_system.ipc_system().step();
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

The rigid-body path remains a classic pre-sync / solve / post-sync flow.

The IPC path is slightly different:

- scene -> IPC happens at registration time, not every frame;
- IPC -> scene happens after every `ipc_system().step()`.

## Current Limits

The bridge is intentionally small:

- it assumes `TetBody` registration is static after setup;
- it does not stream scene transforms back into IPC nodal positions every frame;
- it does not handle contact objects or obstacle bodies yet;
- it expects `IPCTetComponent` and `DeformableMeshComponent` to appear together on the same `GameObject`.

That is enough for the current deformable runtime and the fixed-end example, while keeping the bridge thin and explicit.
