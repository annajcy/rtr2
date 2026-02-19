# RTR2 Architecture Overview   

This document provides an overview of the engine's internal structure and module organization, following the layout of the `src/rtr` directory.

-------

## Editor (`src/rtr/editor`)

ImGui-based tools for scene inspection and engine monitoring.

- **Hierarchy Panel:** Tree view of the World/Scene structure.
- **Inspector Panel:** Property editing for GameObjects and Components.
- **Stats & Logger:** Performance monitoring and real-time log viewing.

### Editor Render (`src/rtr/editor/render`)

Specialized rendering passes for editor-specific overlays (gizmos, grid, etc.).

-------

## App (`src/rtr/app`)

The application layer manages the engine's lifecycle and the main execution loop.

- **Main Entry:** `AppRuntime` coordinates all engine systems.
- **Main Loop:** Handles event processing, updates, and frame scheduling.

## Framework (`src/rtr/framework`)

The core engine logic and scene graph management.

### Core (`src/rtr/framework/core`)

Defines the fundamental building blocks of an RTR2 scene.

- **World:** The top-level container for all objects.
- **Scene:** Manages collections of game objects.
- **GameObject:** The base class for all entity-like objects.

### Component (`src/rtr/framework/component`)

Classic object-oriented component system for extending GameObject functionality.

- **Base:** `Component` base class for all logical traits.
- **Camera & Controls:** Camera components and standard controller logic.
- **Material:** Integration with the rendering resource system.
- **PBPT Integration:** Components specific to the path-tracing bridge (e.g., area lights, specific BSDF data).

### Integration (`src/rtr/framework/integration`)

Bridging logic between different engine modules and external libraries.

- **PBPT Integration:** 
    - `pbpt_scene_importer.hpp`: Loads assets from PBPT's XML format into the engine.
    - `pbpt_scene_export_builder.hpp`: Exports the real-time scene to the path tracer.
    - `pbpt_offline_render_service.hpp`: Logic for triggering and monitoring offline path-traced renders.

### Resource (`src/rtr/framework/resource`)

Framework-level wrappers for resources (meshes, materials) used within the scene graph.

## Resource (`src/rtr/resource`)

The asset management layer.

- **ResourceManager:** Asynchronous loading and caching of textures, meshes, and materials.
- **Resource Types:** Definitions for asset types and their metadata.

## System (`src/rtr/system`)

Higher-level systems that orchestrate specific engine functions.

### Render System (`src/rtr/system/render`)

Manages the Vulkan rendering pipeline, frame scheduling, and resource synchronization.

- **Renderer:** High-level Vulkan renderer control.
- **Frame Scheduler:** Manages multi-buffered rendering and swapchain interaction.
- **Pipelines:** Definition of `IRenderPipeline` and specific implementations.

### Input System (`src/rtr/system/input`)

Handles user input from keyboard, mouse, and gamepads.

- **Input State:** Tracks the current state of all input devices.
- **Input Events:** Propagates input changes to the rest of the engine.

## RHI (`src/rtr/rhi`)

**Rendering Hardware Interface**: Low-level RAII wrappers for Vulkan objects.

- **Context & Device:** Vulkan instance and logical device management.
- **Buffer & Image:** GPU memory resource management.
- **Shader Module:** Loading and handling of compiled Slang/SPIR-V shaders.

## Utils (`src/rtr/utils`)

General-purpose helper functions and boilerplate reduction.

- **Logging:** Custom `spdlog` wrapper.
- **Math Helpers:** Conversion between `pbpt::math` and rendering-friendly formats.
- **Profiling:** Macro-based instrumentation for performance analysis.
