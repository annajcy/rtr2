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

`IPCTetComponent` is the scene-side authoring/cache object for one IPC tet body.

It stores:

- `source_body`: the authoring/source `TetBody` used for re-registration;
- `surface_cache`: the cached `TetSurfaceMapping` from one-time boundary extraction;
- `mesh_cache`: a reusable `ObjMeshData` that can be updated in place every frame.
- dirty/lifecycle flags that tell the scene-sync layer what needs to be pushed into runtime.

### What It Is Not Responsible For

`IPCTetComponent` does not own the render resource, but it does own the tet-side source data and the derived surface cache.

Its constructor derives:

```text
source TetBody
  -> build_tet_surface_mapping(body)
  -> tet_rest_to_surface_mesh(body.geometry, surface)
  -> mesh_cache
```

The render resource is then created from that cached initial mesh:

```text
IPCTetComponent.mesh_cache
  -> create DeformableMesh resource
  -> add DeformableMeshComponent
```

This separation matters because it keeps responsibilities clean:

- `IPCTetComponent` owns the volumetric source data and scene-side caches;
- `DeformableMeshComponent` owns the render-facing mesh handle;
- `sync_scene_to_ipc(...)` performs runtime create/remove/update;
- `sync_ipc_to_scene(...)` writes runtime deformation back into the mesh.

## Registration Flow

The current minimum registration flow for one IPC tet object is:

```text
1. Build TetBody
2. Mark fixed vertices / choose material
3. add IPCTetComponent(std::move(source_body))
5. create DeformableMesh resource from ipc_tet.mesh_cache()
6. add DeformableMeshComponent to the same GameObject
7. let sync_scene_to_ipc(scene, ipc_system) create or update the runtime tet body
```

No external manual `add_tet_body(...)` or `initialize()` call is required anymore. `IPCSystem` rebuilds its global state automatically on the next `step(delta_seconds)` when registration changes.

## `sync_ipc_to_scene(...)`

`sync_ipc_to_scene(scene, ipc_system)` is the per-fixed-tick write-back function.

For each active object:

1. find `IPCTetComponent`;
2. find the colocated `DeformableMeshComponent`;
3. skip if no runtime tet body currently exists for that `GameObject`;
4. look up the tet body by owner id in `IPCSystem`;
5. recover that body's global vertex offset from `body.info.dof_offset / 3`;
6. update `mesh_cache` in place from the global DOF vector;
7. extract positions and normals;
8. call `apply_deformed_surface(...)`.

## Why owner lookup and `dof_offset` both matter

`TetSurfaceMapping.surface_vertex_ids` are **body-local** vertex ids.  
`IPCState::x`, on the other hand, is a **global** concatenated `3N` vector across all bodies.

So the bridge must map:

$$
\text{global vertex id} = \text{body vertex offset} + \text{local surface vertex id}
$$

The scene bridge first resolves "which runtime body belongs to this `GameObject`?" through `IPCSystem`'s owner mapping, and then uses `dof_offset` to answer "where does that body's vertex block begin in the current global state vector?".

This is why `tet_to_mesh.hpp` write-back helpers accept a `vertex_offset` parameter for runtime use.

## `scene_physics_step(...)` Order

The framework-side fixed-step order is now:

```cpp
sync_scene_to_rigid_body(scene, physics_system.rigid_body_system());
sync_scene_to_ipc(scene, physics_system.ipc_system());

physics_system.rigid_body_system().step(dt);
sync_rigid_body_to_scene(scene, physics_system.rigid_body_system());

physics_system.ipc_system().step(dt);
sync_ipc_to_scene(scene, physics_system.ipc_system());
```

The rigid-body path remains a classic pre-sync / solve / post-sync flow.

The IPC path is also a pre-sync / solve / post-sync flow now, but its scene-side input is still limited to source-body/material/cache changes rather than per-frame node transforms.

## Current Limits

The bridge is intentionally small:

- runtime registration is component-driven, but still assumes tet topology is not edited every frame;
- `IPCTetComponent` itself does not hold an `IPCSystem&` or runtime body handle;
- it does not stream scene transforms back into IPC nodal positions every frame;
- it does not handle contact objects or obstacle bodies yet;
- it expects `IPCTetComponent` and `DeformableMeshComponent` to appear together on the same `GameObject`.

That is enough for the current deformable runtime and the fixed-end example, while keeping the bridge thin and explicit.
