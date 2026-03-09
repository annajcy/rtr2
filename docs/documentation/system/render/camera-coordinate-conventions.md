# Camera and Coordinate Conventions

This document records the canonical coordinate-system and camera-transform conventions shared by PBPT and RTR2.

It exists to answer three recurring questions:

1. Which axis is considered "forward"?
2. Which APIs use positive clip distances versus signed camera-space planes?
3. What exact matrix chain is used from object space to the shader and back to editor picking?

## Canonical Convention

The engine uses the following convention end to end:

- Right-handed coordinate system
- Scene/node local axes:
  - `+X`: right
  - `+Y`: up
  - `+Z`: node front
  - `-Z`: node back
- Camera/view space:
  - visible points in front of the camera satisfy `z_view < 0`
  - camera forward is therefore `-Z`
- NDC depth range:
  - near plane maps to `0`
  - far plane maps to `1`

This means "node forward" and "camera forward" are intentionally not the same:

- a regular scene node treats `+Z` as front
- a camera component treats the node's `-Z` direction as camera front

## Node and Camera Axes

Scene-node basis vectors are defined in [`src/rtr/framework/core/scene_graph.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/core/scene_graph.hpp):

- `local_front()` / `world_front()` = rotated `(0, 0, +1)`
- `local_back()` / `world_back()` = rotated `(0, 0, -1)`

Camera components adapt that node convention in [`src/rtr/framework/component/camera/camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/camera.hpp):

- `camera_world_front()` returns `node.world_back()`
- `camera_world_back()` returns `node.world_front()`
- `camera_look_at_*()` negates the requested direction before delegating to the node

That adaptation is deliberate. It lets the scene graph keep a simple `+Z = front` convention while preserving the usual rendering rule that camera-forward points have negative view-space `z`.

## View Matrix Convention

The view matrix is always:

```cpp
view = inverse(camera_world_matrix)
```

See [`src/rtr/framework/component/camera/camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/camera.hpp).

PBPT `look_at(...)` follows the same `-Z forward` camera-space convention in [`external/pbpt/src/pbpt/math/matrix/matrix_transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/math/matrix/matrix_transform.hpp):

- a point directly in front of the camera ends up with `z_view < 0`
- `clip.w = -z_view` after perspective projection

## Near/Far Semantics

There are two layers of API, and they do not use the same units.

### High-level camera APIs use positive distances

These APIs take near/far as positive distances from the camera:

- PBPT camera projection helpers in [`external/pbpt/src/pbpt/camera/camera.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/camera/camera.hpp)
- PBPT XML `near_clip` / `far_clip`
- RTR2 camera components:
  - [`src/rtr/framework/component/camera/perspective_camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/perspective_camera.hpp)
  - [`src/rtr/framework/component/camera/orthographic_camera.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/component/camera/orthographic_camera.hpp)

Examples:

- `near = 0.1`
- `far = 100.0`

### Low-level projection math uses signed camera-space planes

These APIs expect the actual camera-space plane positions:

- [`external/pbpt/src/pbpt/math/matrix/matrix_transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/math/matrix/matrix_transform.hpp)
- [`external/pbpt/src/pbpt/geometry/transform.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/geometry/transform.hpp)

Under the canonical `-Z forward` convention, that means:

- `near_z = -abs(near_distance)`
- `far_z = -abs(far_distance)`

Typical values:

- `near_z = -0.1`
- `far_z = -100.0`

RTR2 camera components do this conversion before calling the low-level PBPT math helpers.

## Projection Rules

### Orthographic

`pbpt::math::orthographic(left, right, bottom, top, near_z, far_z)` assumes:

- `near_z < 0`
- `far_z < near_z`
- near maps to `ndc.z = 0`
- far maps to `ndc.z = 1`

### Perspective

`pbpt::math::perspective(fov_y, aspect, near_z, far_z)` also assumes signed camera-space planes.

Two details matter:

1. `near_z` and `far_z` are used as signed planes for the `z/w` mapping.
2. The frustum width/height must be computed from `abs(near_z)`, not from the signed value itself.

If the frustum extent is computed directly from a negative `near_z`, both `x` and `y` flip, which appears as a 180-degree image rotation.

## Object-to-Screen Matrix Chain

RTR2 uses the following CPU and GPU chain:

```text
object -> world -> view -> clip -> ndc
```

More concretely:

```text
world_pos = model * local_pos
view_pos  = view  * world_pos
clip_pos  = proj  * view_pos
ndc       = clip.xyz / clip.w
```

Relevant files:

- scene collection: [`src/rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp)
- UBO packing: [`src/rtr/system/render/pipeline/forward/forward_pipeline.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/system/render/pipeline/forward/forward_pipeline.hpp)
- shader application: [`shaders/vert_buffer.slang`](https://github.com/annajcy/rtr2/blob/main/shaders/vert_buffer.slang)

## Matrix Storage and Shader Contract

The shader declares:

```slang
row_major float4x4 model;
row_major float4x4 view;
row_major float4x4 proj;
```

and applies them as:

```slang
mul(ubo.model, position)
mul(ubo.view, world_pos)
mul(ubo.proj, view_pos)
```

On the CPU side, [`pack_mat4_row_major(...)`](https://github.com/annajcy/rtr2/blob/main/src/rtr/system/render/pipeline/forward/forward_pipeline.hpp) writes matrix rows directly into the UBO. No transpose compatibility layer is used.

The intended invariant is:

```text
GPU clip result == CPU result of proj * view * model * position
```

## Editor Picking Convention

Editor picking in [`src/rtr/editor/core/scene_picking.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/editor/core/scene_picking.hpp) uses the same projection convention as rendering:

- clip/NDC near depth = `0`
- clip/NDC far depth = `1`
- `inv_view_proj = inverse(proj * view)`

The pick ray is built by unprojecting:

- `(ndc_x, ndc_y, 0, 1)` for the near point
- `(ndc_x, ndc_y, 1, 1)` for the far point

This must stay aligned with the render path. If rendering uses `[0, 1]` depth while picking still assumes `[-1, 1]`, picking will be incorrect even if the viewport image looks fine.

## PBPT Serde Boundary

PBPT XML and RTR2 scene/component APIs expose positive clip distances. The sign conversion happens only when building low-level projection matrices.

Relevant files:

- PBPT XML camera serde: [`external/pbpt/src/pbpt/serde/domain/impl/camera.hpp`](https://github.com/annajcy/pbpt/blob/main/src/pbpt/serde/domain/impl/camera.hpp)
- RTR2 PBPT import: [`src/rtr/framework/integration/pbpt/serde/load/mappers.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/integration/pbpt/serde/load/mappers.hpp)
- RTR2 PBPT export: [`src/rtr/framework/integration/pbpt/serde/scene_writer.hpp`](https://github.com/annajcy/rtr2/blob/main/src/rtr/framework/integration/pbpt/serde/scene_writer.hpp)

The rule is:

- external config and component fields stay positive
- low-level matrix math receives signed negative planes

## Quick Checklist

When adding or debugging camera code, verify these invariants first:

1. Camera front points produce `z_view < 0`.
2. Perspective projection produces `clip.w > 0` for visible points.
3. Near/far public APIs stay positive; only low-level math gets signed planes.
4. NDC depth is `[0, 1]`, not `[-1, 1]`.
5. CPU matrix multiplication, UBO packing, and shader multiplication order all agree.
6. Picking unprojection uses `z = 0/1`, not `-1/1`.

If one stage violates these assumptions, typical symptoms are:

- black screen: clip-space `w` or depth convention mismatch
- image upside down only: viewport or raster-space `y` mismatch
- image both upside down and mirrored: signed near value used to compute frustum width/height
- picking offset or inverted: render path and unprojection depth convention differ
