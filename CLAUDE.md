# CLAUDE.md

This file provides guidance for AI assistants working with the RTR2 codebase.

## Project Overview

RTR2 is a modern real-time 3D rendering engine written in C++23 using Vulkan and Slang shaders. It integrates offline path tracing via the PBPT submodule and explores machine learning techniques for real-time rendering. The math stack uses `pbpt::math` end-to-end (not GLM).

## Build System

**Toolchain:** CMake 3.27+ with Conan 2.0+ package manager, Ninja generator, Python via UV.

### Quick Build (Linux/macOS Debug)

```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug
```

### Running Tests

```bash
# Unit tests only (no GPU required)
ctest --test-dir build/Debug -C Debug -LE integration

# GPU integration tests (requires display or xvfb)
RTR_RUN_GPU_TESTS=1 ctest --test-dir build/Debug -C Debug -L integration
```

### CMake Options

| Option | Default | Description |
|--------|---------|-------------|
| `RTR_BUILD_TESTS` | ON (top-level) | Build test suite |
| `RTR_BUILD_EXAMPLES` | ON (top-level) | Build example applications |
| `RTR_BUILD_EDITOR` | ON | Build ImGui-based editor |
| `RTR_COMPILE_SHADERS` | ON | Compile Slang shaders to SPIR-V |

### Key Build Notes

- Always use `uv run` for Conan/Python commands (no manual venv activation)
- The Conan profile at `profiles/rtr2` sets C++23, Ninja generator, and Embree C++20 override
- Preset names: `conan-debug` / `conan-release` (run `cmake --list-presets` to verify)
- `compile_commands.json` is auto-copied to project root for IDE support
- PBPT is always built from the vendored `external/pbpt` submodule

## Repository Structure

```
rtr2/
├── src/rtr/                    # Main source (header-only C++)
│   ├── app/                    #   Application runtime orchestrator
│   ├── framework/              #   Game engine framework
│   │   ├── core/               #     Engine, World, Scene, SceneGraph, Camera
│   │   ├── component/          #     GameObject components (mesh, camera, pbpt)
│   │   └── integration/pbpt/   #     PBPT offline rendering integration
│   ├── system/                 #   Engine subsystems
│   │   ├── input/              #     Input handling (GLFW)
│   │   └── render/             #     Rendering system & pipelines
│   ├── rhi/                    #   Vulkan abstraction layer (RHI)
│   ├── editor/                 #   ImGui editor panels & overlay
│   ├── resource/               #   Asset management (ResourceManager)
│   └── utils/                  #   Logging, events, file I/O
├── test/                       # GoogleTest suite (~31 tests)
├── examples/                   # Executable demos (4 targets)
├── shaders/                    # Slang shader files (.slang)
├── assets/                     # Models, textures, scenes
├── external/pbpt/              # PBPT submodule (path tracer)
├── cmake/                      # CMake helpers
├── conan_recipe/               # Custom Conan package recipes
├── profiles/                   # Conan build profiles
├── script/                     # Python utility scripts
└── .github/workflows/          # CI pipeline
```

### Architecture Layers

1. **AppRuntime** (`rtr::app`) - Top-level orchestrator coordinating all systems
2. **Framework** (`rtr::framework`) - ECS-like game engine with World/Scene/GameObject
3. **System** (`rtr::system`) - Rendering and input subsystems
4. **RHI** (`rtr::rhi`) - Low-level Vulkan abstraction
5. **Editor** (`rtr::editor`) - ImGui-based in-engine editor UI
6. **Resource** (`rtr::resource`) - Asset lifecycle management
7. **Utils** (`rtr::utils`) - Cross-cutting concerns (logging, events, file I/O)

## Coding Conventions

### Naming

| Element | Convention | Examples |
|---------|-----------|----------|
| Classes/Structs | PascalCase | `AppRuntime`, `EventCenter`, `Renderer` |
| Interfaces | PascalCase with `I` prefix | `IRenderPipeline`, `IEditorPanel` |
| Functions/Methods | snake_case | `init_logging()`, `get_logger()`, `draw_frame()` |
| Member variables | `m_` prefix + snake_case | `m_config`, `m_world`, `m_renderer` |
| Local variables | snake_case | `frame_delta`, `current_time` |
| Enum values | snake_case | `LogLevel::debug`, `LogLevel::critical` |
| Namespaces | snake_case | `rtr::app`, `rtr::system::render` |
| Files | snake_case | `app_runtime.hpp`, `event_center.hpp` |
| Shader variables | camelCase | `iResolution`, `iTime`, `outColor` |
| Shader files | snake_case | `shadertoy_compute.slang` |
| Constants | k + PascalCase | `kDefaultPattern`, `kInvalidSceneId` |

### Header Style

- Use `#pragma once` (not `#ifndef` guards)
- Include order: standard library -> third-party -> project headers (alphabetical within groups)
- Close namespaces with comments: `} // namespace rtr::app`

### Code Style

- **Indentation:** 4 spaces (no tabs)
- **Braces:** Opening brace on same line (`K&R` / `1TBS` style)
- **Line endings:** Unix (`\n`)
- **In-class member initialization:** Use brace initialization (`m_config{}`, `m_world{}`)
- **Struct defaults:** In-class brace-initialized (`std::uint32_t window_width{800};`)

### Architecture Patterns

- **Header-only design:** Most code is in `.hpp` files for compile-time optimization
- **Ownership:** `std::unique_ptr` for exclusive ownership, `std::shared_ptr` for shared (mainly loggers)
- **Non-owning references:** Raw pointers (`rhi::Device*`) for observer/borrowing semantics
- **Move semantics:** Used extensively for config/resource passing
- **Error handling:** Exceptions (`std::runtime_error`, `std::invalid_argument`) with logging before throw
- **Result types:** Structs like `RuntimeResult { bool ok; std::string error_message; }` for operations
- **Logging:** spdlog with per-module categories via `utils::get_logger("module.name")`
- **Events:** Template-based `Event<Args...>` and `EventCenter<Args...>` classes

### Template Patterns

```cpp
// Variadic event types
using KeyEvent = utils::Event<KeyCode, KeyAction, KeyMod>;

// Factory methods with perfect forwarding
template <typename TPanel, typename... TArgs>
TPanel& emplace_panel(TArgs&&... args);
```

## Testing

### Framework and Organization

- **Framework:** GoogleTest (gtest 1.14+)
- **Test CMake functions:** `rtr_add_test()` for unit tests, `rtr_add_integration_test()` for GPU tests
- **Labels:** Unit tests labeled `"unit"`, GPU tests labeled `"integration"` and `"gpu"`

### Test Conventions

- **File naming:** `{feature}_test.cpp` (e.g., `event_center_test.cpp`)
- **Test namespace:** `rtr::{module}::test` (e.g., `rtr::utils::test`)
- **Test naming:** `TEST(PascalCaseClass, PascalCaseMethod)` (e.g., `TEST(EventTest, AddExecuteRemoveByHandle)`)
- **Assertions:** Prefer `EXPECT_*` (non-fatal); use `ASSERT_*` only when continuation is impossible
- **GPU test skipping:** Check `RTR_RUN_GPU_TESTS` env var, use `GTEST_SKIP()` if unavailable
- **Mocks:** Lightweight hand-written probe/stub classes (no mocking framework)
- **Float comparison:** Custom `expect_vec3_near()` / `expect_mat4_near()` helpers

### Adding a New Test

1. Create `test/{category}/{feature}_test.cpp`
2. Place code in `namespace rtr::{module}::test`
3. Add to `test/CMakeLists.txt`:
   ```cmake
   rtr_add_test(test_my_feature {category}/my_feature_test.cpp)
   # or for GPU tests:
   rtr_add_integration_test(test_my_feature_integration {category}/my_feature_integration_test.cpp)
   ```
4. Include `main()` boilerplate:
   ```cpp
   int main(int argc, char** argv) {
       ::testing::InitGoogleTest(&argc, argv);
       return RUN_ALL_TESTS();
   }
   ```

## CI/CD

- **Platforms:** Ubuntu, Windows, macOS
- **Build types:** Debug and Release (6 matrix configurations)
- **Pipeline:** Checkout (with submodules) -> Python/Vulkan SDK setup -> Conan install -> CMake build -> Unit tests -> GPU tests (Linux Debug only, non-blocking)
- **GPU tests** run under `xvfb-run` on Linux and are `continue-on-error: true`
- **Caching:** UV cache and Conan cache with platform+build-type keys

## Key Dependencies

| Library | Purpose |
|---------|---------|
| Vulkan-Loader | GPU graphics API |
| GLFW | Window and input management |
| Slang | Shader language (compiles to SPIR-V) |
| ImGui (docking) | Editor UI framework |
| spdlog | Structured logging |
| pbpt::math | Math library (vectors, matrices, quaternions) |
| Embree | CPU ray tracing (offline rendering) |
| OpenEXR | HDR image format |
| TinyGLTF / tinyobjloader | 3D model loading |
| pugixml | Mitsuba-style scene XML parsing |
| GoogleTest | Testing framework |
| PyTorch / slangtorch | ML integration (Python side) |

## Common Tasks

### Adding a New Component

1. Create header in `src/rtr/framework/component/`
2. Inherit from `Component` base class
3. Register with `GameObject` via `add_component<T>()`
4. Add corresponding test in `test/framework/component/`

### Adding a New Render Pipeline

1. Implement `IRenderPipeline` interface in `src/rtr/system/render/pipeline/`
2. Implement required methods: `prepare()`, `render()`, `cleanup()`
3. Register with `AppRuntime` via `set_pipeline()`

### Adding a New Editor Panel

1. Implement `IEditorPanel` in `src/rtr/editor/`
2. Override `on_frame(EditorContext&)` for ImGui rendering
3. Register via `EditorHost::emplace_panel<T>()`

### Adding a New Shader

1. Create `.slang` file in `shaders/`
2. Use `[[vk_binding(index, set)]]` for Vulkan descriptor bindings
3. Entry points: `[shader("compute")]` with `[numthreads(X, Y, Z)]`
4. Shaders are auto-compiled to SPIR-V during build

## Troubleshooting

- **Stale Conan cache:** `conan remove 'rtr/*' -c` then re-install
- **Missing presets:** Run `cmake --list-presets` to see available names
- **Windows VS detection:** Use `script/setup_conan_profile.ps1` to generate local profile
- **Submodule issues:** `git submodule update --init --recursive`
- **Dependency graph:** `uv run conan graph info . -pr:h=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23`
