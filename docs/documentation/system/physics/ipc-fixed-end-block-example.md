# IPC Fixed-End Block Example

This page explains the minimal editor demo in:

`examples/editor/ipc_fixed_end_block_editor.cpp`

The example is not just a visual demo. It is the first end-to-end runtime path that exercises:

```text
TetBody
  -> IPCSystem.step(dt)
  -> sync_ipc_to_scene()
  -> DeformableMeshComponent
  -> GPU mesh update
```

## Why the Example Uses a Fixed End

The current IPC runtime already has:

- inertial energy
- gravity energy
- tet elastic energy
- Newton solve
- full-vertex Dirichlet constraints through `fixed_vertices`

It does **not** yet have:

- contact / barrier energy
- obstacle coupling
- floor collision

So a free-falling block would simply keep falling through space.  
A fixed-end cantilever is a better minimum example because it:

- shows deformation immediately;
- stays in view;
- does not depend on contact features that are not implemented yet.

## Scene Setup

The demo reuses the standard editor skeleton:

- runtime + editor host
- forward editor pipeline
- one camera
- one point light
- one ground quad for visual reference

The ground is visual only. It is not a collision object.

## Tet Body Construction

The demo uses `generate_tet_block(...)` to create a slender block:

```cpp
auto body = ipc::generate_tet_block(6, 2, 2, 0.2, Eigen::Vector3d(-0.6, 1.4, -0.2));
```

That shape behaves more like a cantilever beam than a cube, so the free end sags more clearly under gravity.

## Fixed-End Constraint

The current runtime uses per-vertex `fixed_vertices`, so the example fixes an entire end face rather than a single vertex.

The logic is:

1. find the minimum `x` in `body.geometry.rest_positions`;
2. mark every vertex with `x == min_x` (within an epsilon) as fixed.

This means all three translational DOFs of that end-face vertex set are constrained.

Why use an end face instead of one point:

- it matches the current DBC representation;
- it is much more stable visually;
- it better matches the intuition of a clamped beam.

## Registration Helper Pattern

The example uses a small local helper that wraps the setup boilerplate:

```text
TetBody
  -> TetSurfaceMapping
  -> initial ObjMeshData
  -> DeformableMesh resource
  -> DeformableMeshComponent
  -> IPCSystem registration
  -> IPCTetComponent
```

This helper is intentionally small. It does not introduce a new engine-wide abstraction; it only keeps the example readable.

## Runtime Behavior

Once the scene is initialized, the example does **not** perform manual mesh write-back inside `on_post_update`.

Instead:

```text
fixed tick
  -> step_scene_physics(...)
      -> ipc_system.step(dt)
      -> sync_ipc_to_scene(...)
```

The callback layer is left with only editor UI duties such as:

- `EditorHost::begin_frame(...)`
- camera aspect-ratio updates
- `ESC` to close the window

That is the key architectural difference from the earlier manual-writeback demo plan.

## Time-Step Alignment

`IPCSystem` now uses the `dt` passed into `step(delta_seconds)` directly, so the example sets:

```cpp
AppRuntimeConfig.fixed_delta_seconds = 0.01;
```

This keeps the runtime fixed tick and the IPC solve step aligned.

## What You Should See

When the demo runs correctly:

- the left end stays fixed in space;
- the free end bends downward under gravity;
- the surface normals remain valid, so lighting stays smooth;
- no manual `tet_to_mesh(...)` call is needed in per-frame user callbacks.

## Current Limits

The demo still reflects the current runtime boundary:

- no collision with the ground;
- no obstacle bodies;
- no contact stabilization;
- no inspector controls for IPC parameters yet.

That is acceptable for this example, because its purpose is to validate the runtime bridge and deformable write-back path, not full physical interaction.
