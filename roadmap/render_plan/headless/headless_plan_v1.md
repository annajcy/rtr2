# RTR2 Headless v1 Detailed Plan

## Scope

This document defines the detailed plan for the **true headless** landing after v0.

`v1` is explicitly **not** a rewrite of the current offline/preview path.

The current swapchain-backed offline path must stay:

- keep `OfflineImageOutputBackend`
- keep `OfflineRuntime`
- keep preview capability
- keep the non-interactive window workflow for users who want to watch the run

`v1` adds a second path:

- no window
- no surface
- no swapchain
- no preview / no present
- offscreen rendering for export only

In other words:

- `v0 OfflineImageOutputBackend` = preview/export backend
- `v1 HeadlessOutputBackend` = true offscreen-only export backend

---

## Goals

v1 is successful when all of the following are true:

1. A true headless runtime can render and export image sequences without creating a window or Vulkan surface.
2. The current `OfflineRuntime + OfflineImageOutputBackend` preview workflow remains available and behaviorally unchanged.
3. Pipelines still only produce `final_output(...)`; no pipeline regains present ownership.
4. Realtime and editor paths continue to compile and run on the existing swapchain-backed renderer path.
5. The new headless path is explicitly non-previewable: no onscreen present, no window polling, no editor UI path.

---

## Non-Goals

v1 explicitly does **not** do the following:

- remove or replace the current preview/offline backend
- merge preview runtime and true headless runtime into one over-generalized type
- reintroduce pipeline-specific editor/realtime variants
- add async PNG writing or staging-ring optimization by default
- add a software renderer or "no rendering at all" path

Note on terminology:

"真正的 headless 没有渲染能力" here should be interpreted as:

- no onscreen rendering / no preview / no present capability
- still performs GPU offscreen rendering in order to export final frames

Without offscreen rendering, there is nothing to export.

---

## Current Constraints From Code

The current codebase still has several surface/swapchain assumptions that block a true headless path.

### 1. `PipelineRuntime` still requires `rhi::Window&`

Current file:

- `src/rtr/system/render/render_pipeline.hpp`

Today `PipelineRuntime` contains:

- `rhi::Device&`
- `rhi::Context&`
- `rhi::Window&`
- swapchain-derived image count / color format / depth format

This means even pure content pipelines still inherit a hard dependency on a windowed renderer bootstrap.

### 2. `RendererT` always creates `Window + Context(surface) + Device + FrameScheduler`

Current file:

- `src/rtr/system/render/renderer.hpp`

Today `RendererT<TBackend>` always owns:

- `rhi::Window`
- `rhi::Context`
- `rhi::Device`
- `FrameScheduler`

and `make_context_create_info(...)` always wires a surface creator through the window.

This is correct for realtime/editor/preview, but it prevents a no-window bootstrap.

### 3. `FrameScheduler` is inherently swapchain-backed

Current file:

- `src/rtr/system/render/frame_scheduler.hpp`

Its job is:

- acquire swapchain image
- submit
- present
- rebuild swapchain

That is exactly what true headless does not need.

### 4. `OfflineImageOutputBackend` is intentionally swapchain-backed

Current file:

- `src/rtr/system/render/output_backend.hpp`

It derives from `SwapchainFrameOutputBackendBase`, which is fine for v0 preview/export.
That backend should remain as-is conceptually. It is not the right starting point for true headless.

### 5. `Context` and `Device` still default to presentation assumptions

Current files:

- `src/rtr/rhi/context.hpp`
- `src/rtr/rhi/device.hpp`

Current blockers:

- `Context` always calls `create_surface()`
- `Device` selection still assumes a presentation-capable queue when a surface is provided
- required device extensions include swapchain-related requirements by default

---

## Core Design Decisions

These decisions should be treated as fixed unless implementation proves a blocker.

### 1. Keep preview backend and true headless backend separate

Do not overload one backend with both responsibilities.

Keep:

- `OfflineImageOutputBackend`
  - swapchain-backed
  - preview-capable
  - used by `OfflineRuntime`

Add:

- `HeadlessOutputBackend`
  - no swapchain
  - no present
  - no preview
  - used by `HeadlessRuntime`

This separation keeps v0 preview behavior stable and avoids forcing preview-specific compromises into the true headless path.

### 2. Keep runtime split explicit

Do not try to force true headless into the existing `OfflineRuntime`.

Keep:

- `OfflineRuntime`
  - preview-oriented offline path
  - may poll window events
  - may present
  - may export frames

Add:

- `HeadlessRuntime`
  - no input system ownership
  - no window event polling
  - no present
  - fixed-count deterministic offline execution

Both runtimes can still share:

- `OfflineFrameTimePolicy`
- `FrameStepper`

### 3. Split renderer bootstrap from output backend

The current renderer template already separates output behavior by backend type, but bootstrap is still hard-wired to the swapchain path.

v1 should split this into two layers:

1. renderer-independent render services bootstrap
2. backend-specific frame orchestration

In practice that means:

- keep current `RendererT<TBackend>` path for swapchain-backed backends
- add a separate headless renderer/executor path for no-surface operation

Do **not** force realtime/editor/preview to go through headless bootstrap abstractions if that makes their path worse.

### 4. Replace window dependence in `PipelineRuntime`

`RenderPipeline` should not require `rhi::Window&` in order to exist.

For v1, `PipelineRuntime` should be refactored so that pipelines receive only what they actually need:

- `rhi::Device&`
- `rhi::Context&`
- render target count / render formats / extent policy
- shader root path

If a pipeline genuinely needs a viewport or framebuffer size source, that should be passed as explicit data, not as a whole window object.

### 5. Headless uses offscreen rendering, not presentation

True headless still renders the scene into offscreen images and exports from `final_output(...)`.

It simply does not:

- create a swapchain image
- present to screen
- maintain any preview output target

---

## Target Architecture

### Path A: Existing preview/export path stays

```text
OfflineRuntime
    -> RendererT<OfflineImageOutputBackend>
    -> Window + Surface + Swapchain + FrameScheduler
    -> Pipeline final_output(...)
    -> Preview present + optional export
```

### Path B: New true headless path

```text
HeadlessRuntime
    -> HeadlessRenderer / HeadlessFrameExecutor
    -> Context(without surface) + Device(headless profile)
    -> Offscreen frame resources + fences + command buffers
    -> Pipeline final_output(...)
    -> Readback / export only
```

### Shared pieces

Both paths should still share:

- `OfflineFrameTimePolicy`
- `FrameStepper`
- pure content pipelines
- `image_readback.hpp`

---

## New Components

### 1. `HeadlessRuntime`

Add:

- `src/rtr/app/headless_runtime.hpp`

Responsibilities:

- validate headless config
- create output directory
- run deterministic fixed-count simulation/render loop
- own the true headless renderer path
- call export-only backend
- never create or poll a window

Config should be intentionally close to `OfflineRuntimeConfig`, but separate:

```cpp
struct HeadlessRuntimeConfig {
    uint32_t width{1920};
    uint32_t height{1080};

    double sim_dt{0.01};
    uint32_t steps_per_output_frame{1};
    uint32_t total_output_frames{500};
    double output_fps{100.0};

    std::filesystem::path output_dir{"output/frames"};
    std::string filename_pattern{"frame_{:06d}.png"};

    bool warn_on_fps_mismatch{true};
    bool fail_on_fps_mismatch{false};
};
```

Do not reuse `OfflineRuntimeConfig` directly.
Preview-only fields and window behavior do not belong in true headless config.

### 2. `HeadlessOutputBackend`

Add:

- `src/rtr/system/render/headless_output_backend.hpp`

Responsibilities:

- consume `pipeline.final_output(frame_index)`
- record readback copy to staging
- submit command buffer
- wait for completion
- finalize readback
- save image

It must **not**:

- derive from `SwapchainFrameOutputBackendBase`
- depend on `FrameScheduler`
- call present
- depend on `RenderOutputTarget` for a swapchain image

### 3. `HeadlessFrameExecutor`

Add:

- `src/rtr/system/render/headless_frame_executor.hpp`

Responsibilities:

- own per-frame command buffers
- own per-frame fences
- provide a `begin_frame / submit_frame` API without swapchain semantics
- expose render extent, frame slot index, and image count to pipelines

This is the no-swapchain analog of the current `FrameScheduler`, but it should not pretend to be the same abstraction.

Recommended per-frame contract:

```cpp
struct HeadlessFrameTicket {
    uint32_t frame_index{0};
    rhi::CommandBuffer* command_buffer{};
    vk::Extent2D render_extent{};
};
```

### 4. Headless renderer/bootstrap

Recommended implementation:

- add `src/rtr/system/render/headless_renderer.hpp`

This renderer/bootstrap should own:

- `rhi::Context` created without surface
- `rhi::Device` created with headless profile
- `HeadlessFrameExecutor`
- `HeadlessOutputBackend`

Do not try to force this into the existing `RendererT<TBackend>` if that requires fake windows or fake frame schedulers.

---

## Required Refactors

## Step 1: Make `Context` support no-surface mode

### Files

- `src/rtr/rhi/context.hpp`

### Changes

Refactor `ContextCreateInfo` to make surface creation optional in a first-class way.

Suggested shape:

```cpp
struct ContextCreateInfo {
    std::string app_name{"RTR"};
    std::vector<std::string> instance_extensions{};
    std::function<std::optional<VkSurfaceKHR>(const vk::raii::Instance&)> surface_creator{};
    bool require_surface{true};
    bool enable_validation_layers{...};
};
```

Rules:

- `require_surface = true`
  - current behavior
- `require_surface = false`
  - do not call `create_surface()`
  - do not require surface extensions
  - allow `surface()` to be empty

Also split instance extension policy:

- presentation profile extensions
- headless/offscreen profile extensions

Do not keep surface-only instance extensions enabled by default in headless mode.

### Acceptance

- swapchain-backed paths still create instance + surface exactly as before
- headless path can create a Vulkan instance with no surface

## Step 2: Add device profiles and select queue families without present support

### Files

- `src/rtr/rhi/device.hpp`

### Changes

Introduce an explicit device creation/profile concept, for example:

```cpp
enum class DeviceProfile {
    RealtimePresentation,
    HeadlessOffscreen,
};
```

Or an equivalent descriptor struct if that proves cleaner.

Headless profile requirements:

- graphics queue required
- compute queue required if current renderer/pipelines assume it
- transfer queue support via same family is acceptable
- no surface support required
- no swapchain extension required
- no swapchain maintenance feature required

Realtime profile keeps current requirements.

The selector must only call `getSurfaceSupportKHR(...)` when a surface is actually required.

### Acceptance

- realtime/editor/preview still pick a presentation-capable device
- headless can initialize on a device without binding a surface

## Step 3: Remove `rhi::Window&` from `PipelineRuntime`

### Files

- `src/rtr/system/render/render_pipeline.hpp`
- any pipeline constructors that currently depend on `runtime.window`

### Changes

Refactor `PipelineRuntime` so it no longer stores `rhi::Window&`.

Recommended replacement fields:

- `vk::Extent2D initial_render_extent`
- `uint32_t image_count`
- `vk::Format color_format`
- `vk::Format depth_format`
- shader root path

If any pipeline still needs window-derived values, pass those values explicitly as data.

This is the critical change that allows the same pure content pipeline class to be constructed for both:

- swapchain-backed renderer
- true headless renderer

### Acceptance

- `ForwardPipeline` and other pure pipelines no longer require a window object to construct
- editor/realtime paths still receive the data they need

## Step 4: Introduce a no-swapchain frame executor

### Files

- `src/rtr/system/render/headless_frame_executor.hpp`

### Changes

Implement a frame executor with:

- one command pool
- `kFramesInFlight` command buffers
- `kFramesInFlight` fences
- simple `begin_frame() / submit_frame()` flow

No acquire/present/recreate behavior.

The executor only rotates frame slots and waits on per-frame fences.

### Acceptance

- headless path can record and submit repeated frames with deterministic frame slot rotation
- no swapchain objects are created or referenced

## Step 5: Implement `HeadlessOutputBackend`

### Files

- `src/rtr/system/render/headless_output_backend.hpp`

### Changes

Implement a backend that:

1. begins a headless frame
2. builds a `FrameContext` with:
   - command buffer
   - render extent
   - frame index
   - no output target
3. calls `pipeline.render(frame_ctx)`
4. consumes `pipeline.final_output(frame_index)`
5. records readback copy
6. submits
7. waits
8. finalizes readback and writes PNG

Important rule:

- export from `final_output(...)`
- do not invent a fake swapchain target
- do not perform present-style transitions

### Acceptance

- the headless backend can export PNGs with no window and no swapchain

## Step 6: Add `HeadlessRuntime`

### Files

- `src/rtr/app/headless_runtime.hpp`

### Changes

Use the same offline deterministic loop semantics as `OfflineRuntime`, but remove preview concerns:

- no `poll_events()`
- no window object
- no preview backend

Runtime loop should stay:

1. validate config
2. create output directory
3. `on_startup`
4. for each output frame
   - `plan = OfflineFrameTimePolicy.make_plan(false)`
   - `on_pre_update`
   - `FrameStepper::execute(plan, ...)`
   - `on_post_update`
   - `pipeline.prepare_frame(...)`
   - `on_pre_render`
   - `headless_renderer.draw_frame()`
   - `on_post_render`
   - `resources.tick(frame_serial)`
   - increment `frame_serial`
5. `on_shutdown`
6. `device.wait_idle()`
7. `resources.flush_after_wait_idle()`

### Acceptance

- headless runtime runs without window creation
- update semantics match offline preview runtime

---

## Example and Tooling

### New example

Add:

- `examples/headless/ipc_true_headless_render.cpp`

This example should:

- use `HeadlessRuntime`
- export frames by default
- never create a preview window

Keep existing examples:

- `examples/headless/ipc_offline_preview.cpp`
- `examples/headless/ipc_offline_render.cpp`

These remain the preview-capable v0 path.

### Video helper

Retain:

- `examples/headless/make_video.py`

No change in ownership: headless runtime exports images only; video remains a manual post-process.

---

## Test Plan

### Runtime/behavior

Add focused tests for `HeadlessRuntime`:

- runs `total_output_frames` exactly
- fixed tick count equals `total_output_frames * steps_per_output_frame`
- uses no wall-clock
- creates no window

### Bootstrap

Add focused tests or at least smoke coverage for:

- `Context` without surface
- `Device` headless profile initialization
- headless frame executor frame rotation

### Export path

Manual or automated smoke checks:

- output directory created
- correct number of frames exported
- first frame exists
- last frame exists
- filenames follow `frame_{:06d}.png`

### Regression checks

Rebuild and smoke-check:

- realtime path
- editor path
- preview offline path

The key regression guarantee is:

- `OfflineImageOutputBackend` still works
- `OfflineRuntime` still provides preview
- true headless is additive, not a replacement

---

## Milestones

### Milestone 1: Bootstrap split

- `Context` supports no-surface mode
- `Device` supports headless profile
- `PipelineRuntime` no longer requires window

### Milestone 2: Headless rendering path

- `HeadlessFrameExecutor`
- `HeadlessOutputBackend`
- headless renderer/bootstrap

### Milestone 3: End-to-end runtime and example

- `HeadlessRuntime`
- `ipc_true_headless_render`
- export smoke test

---

## Acceptance Criteria

v1 is done when all of the following are true:

1. `OfflineRuntime + OfflineImageOutputBackend` still provide the existing preview-capable workflow.
2. A new `HeadlessRuntime` can export image sequences with no window, no surface, and no swapchain.
3. Pipelines remain pure producers of `final_output(...)`.
4. Realtime/editor codepaths do not regress.
5. The distinction between preview/offline and true headless is explicit in both code and documentation.
