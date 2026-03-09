# Profiling RTR2 on macOS

This guide shows how to capture the first frame from `quickstart` on macOS and use the capture to locate a slow pass.

`RTR2` is a Vulkan renderer. On macOS it runs through MoltenVK, so the practical workflow is:

1. Use MoltenVK to emit a `.gputrace` capture.
2. Open that capture in Xcode and inspect the frame.
3. Use Instruments `Metal System Trace` to confirm where frame time is going.

## 1. Prerequisites

- macOS with Xcode installed
- Vulkan SDK / loader available on the machine
- `quickstart` already built

If you have not built the project yet:

```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug --target quickstart
```

Sanity-check that Vulkan is reaching MoltenVK:

```bash
vulkaninfo --summary
```

You should see:

- `driverName = MoltenVK`
- a macOS or Metal surface extension such as `VK_EXT_metal_surface`

## 2. Know what `quickstart` renders

The `quickstart` executable is:

```bash
./build/Debug/examples/quickstart
```

Its entry point is `examples/editor/quickstart.cpp`.

That sample uses `ForwardEditorPipeline`, not the plain runtime `ForwardPipeline`. The frame sequence is:

1. `ForwardPass`: render the 3D scene into an offscreen color target
2. image barriers: offscreen target becomes shader-readable, swapchain becomes color-attachment
3. `EditorImGuiPass`: draw the editor UI onto the swapchain, with the scene view sampling the offscreen image

When you inspect the capture, expect to see both the scene render and the ImGui/editor overlay.

## 3. Capture the first frame

From the repository root:

```bash
cd /Users/jinceyang/Desktop/codebase/graphics/rtr2
```

Run:

```bash
MTL_CAPTURE_ENABLED=1 \
MVK_CONFIG_AUTO_GPU_CAPTURE_SCOPE=2 \
MVK_CONFIG_AUTO_GPU_CAPTURE_OUTPUT_FILE=/tmp/rtr2-quickstart-first-frame.gputrace \
./build/Debug/examples/quickstart
```

Notes:

- `MTL_CAPTURE_ENABLED=1` enables Metal frame capture.
- `MVK_CONFIG_AUTO_GPU_CAPTURE_SCOPE=2` asks MoltenVK to capture a single frame automatically.
- `MVK_CONFIG_AUTO_GPU_CAPTURE_OUTPUT_FILE=...` writes the result to a file you can open later.

After the app launches and exits the captured frame, open the trace:

```bash
open /tmp/rtr2-quickstart-first-frame.gputrace
```

Xcode should open the GPU trace directly.

## 4. Read the first frame in Xcode

In Xcode GPU Frame Capture:

1. Open the command buffer or frame summary view.
2. Find the render pass that draws the offscreen scene.
3. Find the later render pass that draws ImGui onto the swapchain.
4. Expand encoders and draw calls to see where the GPU time clusters.

For `quickstart`, use this mapping:

- Scene rendering time:
  `ForwardEditorPipeline::render()` calling `m_forward_pass.execute(...)`
- Barrier section:
  `pipelineBarrier2(...)` between the scene pass and editor pass
- UI / overlay time:
  `ForwardEditorPipeline::render()` calling `m_editor_pass.execute(...)`

If one encoder is much longer than the others, that is your first suspect pass.

## 5. Map a slow region back to source

Use this checklist when the Xcode capture shows a slow pass.

### Slow scene pass

Symptoms:

- Most GPU time is spent before the swapchain/UI pass
- Draw-heavy encoder
- Fragment or bandwidth counters dominate

Start from:

- `src/rtr/editor/render/forward_editor_pipeline.hpp`
- `src/rtr/system/render/pipeline/forward/forward_pass.hpp`
- `src/rtr/system/render/pipeline/forward/forward_scene_view_builder.hpp`

Questions to ask:

- Is the scene target larger than expected because of Retina scaling?
- Are too many meshes or draw items being submitted?
- Is fragment shading expensive relative to this simple scene?
- Are large attachments or depth targets being recreated too often?

### Slow editor / ImGui pass

Symptoms:

- The second render pass is unexpectedly expensive
- GPU time rises when docking, resizing, or showing many panels

Start from:

- `src/rtr/editor/render/editor_imgui_pass.hpp`

Questions to ask:

- Is the scene panel texture being refreshed more often than expected?
- Is UI overdraw dominating because the scene window covers most of the screen?
- Does the cost disappear if you minimize editor interaction or reduce window size?

### Slow present path or end-of-frame stall

Symptoms:

- GPU work is short, but frame time is still long
- Large idle gaps appear near present

Start from:

- `src/rtr/system/render/renderer.hpp`
- `src/rtr/system/render/frame_scheduler.hpp`

Questions to ask:

- Is the app blocked on acquire/present rather than shading?
- Did a resize trigger swapchain recreation?
- Are validation or capture tools distorting timing?

## 6. Confirm with Instruments

Xcode frame capture is good for locating a suspicious pass. Use Instruments to confirm whether it is actually driving frame time.

Steps:

1. Open Instruments.
2. Choose `Metal System Trace`.
3. Launch `./build/Debug/examples/quickstart`.
4. Record 3 to 5 seconds while rotating the camera or interacting with the editor.
5. Stop and inspect the CPU and GPU lanes.

Look for:

- long GPU encoders matching the pass you saw in Xcode
- gaps where the CPU is not feeding work
- gaps where the GPU is idle waiting for present or synchronization
- spikes when the framebuffer size changes

If the long frame in Instruments matches the same pass from the one-frame capture, you have a real hotspot instead of a capture artifact.

## 7. Reduce false signals

Before trusting numbers:

- Prefer `Release` for performance conclusions
- Treat `Debug` timings as directional only
- Avoid resizing the window while recording unless resize cost is the subject
- Keep the test scene stable across runs

Important caveats:

- Debug builds may enable Vulkan validation and significantly distort timing
- Metal frame capture itself adds overhead
- `quickstart` includes editor UI, so it is not a pure scene-only benchmark

## 8. A practical first workflow

Use this exact loop:

1. Capture one frame from `quickstart`
2. Decide whether the slow region is scene, barrier/sync, or ImGui
3. Open the corresponding source area
4. Form one hypothesis
5. Re-run with Instruments to see whether the change moves frame time

Examples of good first hypotheses:

- the offscreen scene target is too large on Retina displays
- fragment work in the scene pass dominates this sample
- editor UI cost is hiding the real scene cost
- swapchain recreation is happening during interaction

## 9. Optional: capture Vulkan API traffic

If you need Vulkan-level replay instead of Apple GPU tools, use `gfxreconstruct`:

```bash
VK_INSTANCE_LAYERS=VK_LAYER_LUNARG_gfxreconstruct \
GFXRECON_CAPTURE_FILE=/tmp/rtr2-quickstart.gfxr \
./build/Debug/examples/quickstart
```

This is useful for API-level debugging and resource lifetime inspection, but for pass-level GPU profiling on macOS, Xcode and Instruments are usually the better first tools.
