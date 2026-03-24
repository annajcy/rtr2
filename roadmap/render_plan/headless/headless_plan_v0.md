# RTR2 Headless v0 Detailed Plan

## Scope

This document defines the detailed implementation plan for the first usable offline/headless landing.

The scope here is **v0 only**:

- keep GLFW / surface / swapchain alive
- create a non-interactive but non-minimized window
- do not implement true no-surface Vulkan yet
- make offline rendering deterministic and clock-free
- export image sequences from the final offscreen image

This plan is derived from [headless_plan.md](/Users/jinceyang/Desktop/codebase/graphics/rtr2/roadmap/render_plan/headless/headless_plan.md), but narrows the target to the first end-to-end working version.

---

## v0 Goals

v0 is successful when all of the following are true:

1. An `OfflineRuntime` can run a scene for a fixed number of output frames without using wall-clock time.
2. Physics and world update order remain consistent with the shared runtime step semantics.
3. `ForwardPipeline` produces a final offscreen image that can be consumed by an output backend.
4. An offline output backend can read back the final image and save PNG files.
5. An example can render an IPC scene to a frame directory and then turn it into video with `ffmpeg`.

---

## Non-Goals

v0 explicitly does **not** do the following:

- true no-window / no-surface Vulkan initialization
- generic trait-based runtime templates
- multi-threaded PNG writing
- staging ring optimization
- editor runtime refactor beyond what is necessary to coexist with the new backend split
- exact parity with every `AppRuntime` callback; `OfflineRuntime` keeps a minimal callback surface

---

## Core Design Decisions

These are fixed for v0 and should not be reopened during implementation unless blocked by the codebase.

### 1. No wall-clock in offline mode

`OfflineRuntime` must not call any real-time clock to decide simulation delta.

Per output frame:

- run `steps_per_output_frame` fixed steps
- each fixed step uses `sim_dt`
- variable update delta is `sim_dt * steps_per_output_frame`

### 2. No accumulator / catch-up in offline mode

Realtime keeps:

- previous time
- accumulator
- max frame delta clamp
- max fixed steps cap

Offline mode does not use any of these.

### 3. Pipeline only produces final offscreen image

Pipeline ownership ends at:

- rendering the final offscreen image
- exposing that image and its extent/layout to the caller

Pipeline does not own:

- present
- readback
- export

### 4. Backend owns output

Backend owns:

- acquire / present for realtime
- editor compose path for editor mode
- readback / PNG export for offline mode

### 5. Offline runtime keeps minimal host API

`OfflineRuntime` keeps:

- `on_startup`
- `on_pre_update`
- `on_post_update`
- `on_pre_render`
- `on_post_render`
- `on_shutdown`

`OfflineRuntime` does not keep:

- `on_input`
- window event polling semantics
- default input begin/end handling

### 6. v0 window strategy

Use a non-interactive but non-minimized window.

Reason:

- v0 still depends on GLFW / surface / swapchain
- minimizing/hiding may create unstable swapchain/extents on some platforms
- this is a correctness-first landing

---

## Deliverables

v0 should produce these deliverables:

1. Shared frame step abstractions:
   - `frame_time_policy.hpp`
   - `frame_stepper.hpp`
2. Offline runtime:
   - `offline_runtime.hpp`
3. Output backend split:
   - base backend interface
   - realtime backend
   - editor backend
   - offline image backend
4. Pipeline/output handoff:
   - `FrameContext` no longer hard-requires swapchain image
   - `ForwardPipeline` exposes final offscreen image
5. Readback utility:
   - `image_readback.hpp`
6. Example:
   - `examples/headless/ipc_offline_render.cpp`
7. Support script:
   - `examples/headless/make_video.sh`

---

## Proposed File Plan

### New files

- `src/rtr/app/frame_time_policy.hpp`
- `src/rtr/app/frame_stepper.hpp`
- `src/rtr/app/offline_runtime.hpp`
- `src/rtr/system/render/output_backend.hpp`
- `src/rtr/system/render/editor_output_backend.hpp`
- `src/rtr/system/render/offline_image_output_backend.hpp`
- `src/rtr/rhi/image_readback.hpp`
- `examples/headless/ipc_offline_render.cpp`
- `examples/headless/make_video.sh`

### Files likely modified

- `src/rtr/app/app_runtime.hpp`
- `src/rtr/system/render/renderer.hpp`
- `src/rtr/system/render/frame_context.hpp`
- `src/rtr/system/render/pipeline/forward/forward_pipeline.hpp`
- `src/rtr/system/render/pass/present_pass.hpp`
- `examples/CMakeLists.txt`

### Files that should remain untouched in v0 unless blocked

- `src/rtr/rhi/context.hpp`
- `src/rtr/rhi/device.hpp`
- `src/rtr/rhi/window.hpp`

If v0 starts requiring changes there, stop and re-evaluate. That means scope is drifting toward v1.

---

## Work Breakdown

## Step 1: Extract shared time policy and stepper

### Goal

Separate:

- how a frame decides time
- how a frame executes updates

### New interfaces

```cpp
struct FrameExecutionPlan {
    uint32_t fixed_steps_to_run{0};
    double fixed_dt{0.0};
    double frame_delta_seconds{0.0};
};
```

```cpp
class RealtimeFrameTimePolicy {
public:
    explicit RealtimeFrameTimePolicy(
        double fixed_delta_seconds,
        uint32_t max_fixed_steps_per_frame,
        double max_frame_delta_seconds
    );

    FrameExecutionPlan make_plan(double current_time_seconds, bool paused);
    void reset(double current_time_seconds);
};
```

```cpp
class OfflineFrameTimePolicy {
public:
    explicit OfflineFrameTimePolicy(
        double sim_dt,
        uint32_t steps_per_output_frame
    );

    FrameExecutionPlan make_plan(bool paused) const;
};
```

```cpp
struct FrameStepperContext {
    framework::core::World& world;
    system::physics::PhysicsSystem& physics_system;
    std::uint64_t& fixed_tick_index;
    std::uint64_t frame_serial;
};
```

```cpp
class FrameStepper {
public:
    void run_fixed_steps(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const;
    void run_variable_update(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const;
    void execute(const FrameExecutionPlan& plan, FrameStepperContext& ctx) const;
};
```

### Refactor target in `AppRuntime`

Refactor `AppRuntime::run()` so that:

- realtime wall-clock logic stays in `RealtimeFrameTimePolicy`
- shared update semantics move into `FrameStepper`
- input begin/end and `resources.tick(...)` stay owned by `AppRuntime`

### Detailed tasks

1. Introduce `FrameExecutionPlan`.
2. Move accumulator logic out of the main body of `AppRuntime::run()`.
3. Convert the existing fixed-step loop into `FrameStepper::run_fixed_steps(...)`.
4. Convert `world.tick(...)` + `world.late_tick(...)` into `FrameStepper::run_variable_update(...)`.
5. Keep existing callback order unchanged in realtime path.

### Done when

- `AppRuntime` runtime behavior is unchanged
- realtime still uses wall-clock accumulator
- shared step execution compiles independently of window/input code

---

## Step 2: Introduce output backend abstraction in `Renderer`

### Goal

Stop hardcoding acquire/render/present into a single frame path.

### New interfaces

```cpp
struct RenderOutputTarget {
    const vk::raii::ImageView* image_view{};
    const vk::Image* image{};
    vk::ImageLayout expected_layout{vk::ImageLayout::eUndefined};
};
```

```cpp
struct RenderFrameTicket {
    uint32_t frame_index{0};
    rhi::CommandBuffer* command_buffer{};
    vk::Extent2D render_extent{};
    std::optional<RenderOutputTarget> output_target{};
};
```

```cpp
class RenderOutputBackend {
public:
    virtual ~RenderOutputBackend() = default;
    virtual std::optional<RenderFrameTicket> begin_frame() = 0;
    virtual void end_frame(RenderFrameTicket& ticket) = 0;
};
```

### v0 concrete backends

```cpp
class SwapchainOutputBackend final : public RenderOutputBackend { ... };
class EditorOutputBackend final : public RenderOutputBackend { ... };
class OfflineImageOutputBackend final : public RenderOutputBackend { ... };
```

`EditorOutputBackend` should be introduced in v0 now.  
The goal is not to finish a broad editor refactor, but to ensure `Renderer` does not keep editor behavior as an implicit special case outside the backend abstraction.

### Renderer target shape

```cpp
class Renderer {
public:
    enum class OutputMode {
        Realtime,
        Editor,
        Offline,
    };

    void set_output_mode(OutputMode mode);
    void draw_frame();
    void render_offline_frame();
};
```

Implementation guideline:

- both `draw_frame()` and `render_offline_frame()` should delegate to a shared helper
- the shared helper should receive a `RenderOutputBackend&`

### Detailed tasks

1. Create output backend base interface.
2. Move current frame scheduler acquire/present flow behind `SwapchainOutputBackend`.
3. Introduce `EditorOutputBackend` in the same abstraction layer, even if its first implementation is thin.
4. Keep realtime path behavior identical after the move.
5. Add `Renderer::render_offline_frame()` that uses offline backend plumbing.

### Done when

- `Renderer` no longer assumes swapchain output in its core record/submit path
- realtime still renders as before
- editor path is explicitly represented by `EditorOutputBackend`
- offline mode has a place to plug in readback/export later

---

## Step 3: Decouple `FrameContext` and `ForwardPipeline` from swapchain ownership

### Goal

Make the renderer/pipeline boundary compatible with non-swapchain output.

### `FrameContext` target shape

```cpp
class FrameContext {
public:
    FrameContext(
        rhi::Device* device,
        rhi::CommandBuffer* cmd,
        const vk::Extent2D& render_extent,
        uint32_t frame_index,
        std::optional<RenderOutputTarget> output_target = std::nullopt
    );

    rhi::CommandBuffer& cmd();
    const vk::Extent2D& render_extent() const;
    uint32_t frame_index() const;

    bool has_output_target() const;
    const RenderOutputTarget& output_target() const;
};
```

### `ForwardPipeline` target shape

```cpp
class ForwardPipeline final : public RenderPipeline {
public:
    void prepare_frame(const FramePrepareContext& ctx) override;
    void render(FrameContext& ctx) override;

    const FrameTrackedImage& final_color_image(uint32_t frame_index) const;
    vk::Extent2D final_image_extent() const;
};
```

### v0 pipeline behavior

`ForwardPipeline::render(...)` should:

1. render scene into offscreen color/depth
2. stop there

It should no longer:

- present
- read back
- save images

If realtime still needs swapchain output, that output should happen after pipeline render, in the realtime backend.

### Detailed tasks

1. Remove swapchain hard requirement from `FrameContext`.
2. Change `PresentPass` ownership so it becomes backend-side or realtime-only utility.
3. Make `ForwardPipeline` expose final color image handle, layout, extent.

### Done when

- `ForwardPipeline` no longer owns the output action
- offline backend can discover the final color image without poking into unrelated internals

---

## Step 4: Implement `OfflineRuntime`

### Goal

Provide a frame-count-driven runtime with deterministic simulation progression.

### Config

```cpp
struct OfflineRuntimeConfig {
    uint32_t width{1920};
    uint32_t height{1080};

    double sim_dt{0.01};
    uint32_t steps_per_output_frame{1};
    uint32_t total_output_frames{500};
    double output_fps{100.0};

    std::filesystem::path output_dir{"output/frames"};
    std::string filename_pattern{"frame_{:06d}.png"};

    bool create_non_interactive_window{true};
    bool warn_on_fps_mismatch{true};
    bool fail_on_fps_mismatch{false};
};
```

### Validation

```cpp
struct OfflineConfigValidationResult {
    bool ok{true};
    std::string error_message{};
    std::vector<std::string> warnings{};
};
```

Check:

```cpp
expected_fps = 1.0 / (sim_dt * steps_per_output_frame)
```

If mismatch:

- warn by default
- optionally fail if configured to do so

### Callback surface

```cpp
struct OfflineRuntimeCallbacks {
    std::function<void(OfflineRuntimeContext&)> on_startup{};
    std::function<void(OfflineRuntimeContext&)> on_pre_update{};
    std::function<void(OfflineRuntimeContext&)> on_post_update{};
    std::function<void(OfflineRuntimeContext&)> on_pre_render{};
    std::function<void(OfflineRuntimeContext&)> on_post_render{};
    std::function<void(OfflineRuntimeContext&)> on_shutdown{};
};
```

### Runtime skeleton

```cpp
class OfflineRuntime {
public:
    explicit OfflineRuntime(OfflineRuntimeConfig config = {});

    framework::core::World& world();
    resource::ResourceManager& resource_manager();
    system::render::Renderer& renderer();
    system::physics::PhysicsSystem& physics_system();

    void set_pipeline(std::unique_ptr<system::render::RenderPipeline> pipeline);
    void set_callbacks(OfflineRuntimeCallbacks callbacks);

    void request_stop();
    OfflineRuntimeResult run();
};
```

### Required runtime order

Per output frame:

1. build plan from `OfflineFrameTimePolicy`
2. `on_pre_update`
3. `FrameStepper::execute(plan, ...)`
4. `on_post_update`
5. `pipeline->prepare_frame(...)`
6. `on_pre_render`
7. `renderer.render_offline_frame()`
8. `on_post_render`
9. `resources.tick(frame_serial)`
10. increment frame serial

### Explicit v0 rule

Do not call:

- `input.begin_frame()`
- `input.end_frame()`
- `window.poll_events()`
- any wall-clock time query

### Done when

- runtime completes a fixed number of output frames
- fixed ticks count matches `total_output_frames * steps_per_output_frame`
- no input/event dependency exists in the default path

---

## Step 5: Implement image readback and offline output backend

### Goal

Make backend-owned export work end-to-end.

### Readback utility

```cpp
struct ReadbackImageDesc {
    vk::Image image{};
    vk::Format format{vk::Format::eUndefined};
    vk::Extent2D extent{};
    vk::ImageLayout current_layout{vk::ImageLayout::eUndefined};
};

struct ReadbackResult {
    std::vector<std::uint8_t> pixels;
    uint32_t width{0};
    uint32_t height{0};
    vk::Format format{vk::Format::eUndefined};
};
```

```cpp
ReadbackResult readback_image(
    rhi::Device& device,
    const ReadbackImageDesc& desc
);

void save_png(
    const std::filesystem::path& output_path,
    const ReadbackResult& image
);
```

### Backend responsibilities

`OfflineImageOutputBackend` should:

1. begin a frame
2. provide command buffer / frame index
3. after pipeline render, locate the pipeline final image
4. record copy/readback
5. submit and wait
6. save PNG with configured naming pattern

### Important simplification for v0

Use a synchronous path:

- render
- copy
- submit
- wait
- save

Do not optimize yet.

### Done when

- PNG files are written in the expected directory
- file names follow the configured pattern
- there is no readback logic inside the pipeline

---

## Step 6: Build the IPC example

### Goal

Prove the full v0 workflow on a real scene.

### Example requirements

`examples/headless/ipc_offline_render.cpp` should:

1. create `OfflineRuntimeConfig`
2. create `OfflineRuntime`
3. build scene
4. set camera / lights / objects
5. bind `ForwardPipeline`
6. call `run()`

The example should prefer reusing existing scene setup helpers where possible, not duplicating editor-only code.

### Script

`examples/headless/make_video.sh` should accept:

- frame directory
- optional output path
- optional fps

### Done when

- one command produces frames
- one command converts them to video

---

## Suggested Implementation Order

Follow this exact order unless blocked:

1. Step 1
2. Step 2
3. Step 3
4. Step 4
5. Step 5
6. Step 6

Reason:

- Step 1 stabilizes update semantics
- Step 2 stabilizes renderer/backend boundary
- Step 3 stabilizes pipeline/output ownership
- Step 4 and Step 5 can then be wired without reworking the lower layers

---

## Validation Checklist

### A. Shared step semantics

- realtime still behaves identically
- fixed tick count is unchanged for realtime
- no duplicate `resources.tick(...)`

### B. Offline time semantics

- no wall-clock query in offline frame progression
- no accumulator in offline progression
- `frame_delta_seconds == sim_dt * steps_per_output_frame`

### C. Output ownership

- pipeline never saves images
- offline backend is the only PNG/export owner
- realtime present still works

### D. End-to-end result

- output frames are deterministic across runs
- output frame count matches config
- first and last frame paths are correct

---

## Exit Criteria

v0 is complete when:

1. `OfflineRuntime` runs deterministically without clock/input dependency.
2. `ForwardPipeline` produces a backend-consumable final offscreen image.
3. `OfflineImageOutputBackend` exports PNG files correctly.
4. An IPC example demonstrates the full workflow.
5. The original realtime path still works.

---

## Risks

### 1. Renderer refactor leaks swapchain assumptions

Mitigation:

- keep backend boundary thin
- move logic, do not redesign unrelated systems

### 2. Pipeline/backend ownership becomes blurry again

Mitigation:

- enforce one rule: pipeline produces image, backend outputs it

### 3. Offline runtime accidentally reintroduces realtime semantics

Mitigation:

- ban wall-clock from `OfflineRuntime`
- validate no accumulator exists in offline path

### 4. Window mode behaves inconsistently across platforms

Mitigation:

- v0 uses non-interactive but non-minimized window
- if that becomes unstable, stop and reassess before touching v1 scope
