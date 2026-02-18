# GEMINI.md - Project Context for RTR2

## Project Overview
RTR2 (Real-Time Renderer 2) is a modern rendering engine built with **C++23**, **Vulkan 1.3** (using `vk::raii`), and **Slang** shading language. It is designed to be a successor to RTR, focusing on modern GPU features (dynamic rendering, sync2, etc.) and exploring the integration of path-tracing (via PBPT) and machine learning into real-time rendering.

### Key Technologies
- **Rendering API:** Vulkan 1.3 (via `vulkan-hpp` RAII wrappers).
- **Shading Language:** Slang (pre-compiled to SPIR-V).
- **Dependency Management:** Conan 2.0+ (managed via `uv` for Python environment).
- **Build System:** CMake 3.27+ with Ninja generator.
- **UI:** ImGui (docking branch) for the editor and runtime overlays.
- **Math Library:** `pbpt::math` (from the vendored PBPT path tracer).
- **Logging:** `spdlog` with a custom thin wrapper in `rtr::utils`.
- **Testing:** GoogleTest.

## Architecture
The project is divided into several logical layers in `src/rtr`:
1.  **RHI (Rendering Hardware Interface):** Low-level wrappers for Vulkan objects (`Context`, `Device`, `Buffer`, `Image`, `CommandBuffer`).
2.  **System/Render:** High-level renderer management, swapchain handling (`FrameScheduler`), and render pipelines (`IRenderPipeline`).
3.  **Framework:** Core engine logic including `World`, `Scene`, `GameObject`, and `Component` (ECS-like but more classic OO-based hierarchy).
4.  **Resource:** `ResourceManager` for handling asynchronous and synchronous loading of meshes, textures, and materials.
5.  **App:** `AppRuntime` provides the main engine loop, managing the lifecycle of all systems and feeding data into the renderer.
6.  **Editor:** ImGui-based editor components (Hierarchy, Inspector, Stats, Logger).

## Building and Running

### Prerequisites
- Python and `uv` installed.
- Vulkan SDK 1.3+.
- A C++23-compliant compiler (Clang 16+, GCC 13+, MSVC 2022+).

### Build Commands
1.  **Setup Environment:**
    ```bash
    uv sync
    ```
2.  **Register Local Slang Recipe:**
    ```bash
    cd conan_recipe
    uv run python build_conan_recipes.py -d . -b Debug -v
    cd ..
    ```
3.  **Install Dependencies:**
    ```bash
    uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
    ```
4.  **Configure and Build:**
    ```bash
    cmake --preset conan-debug
    cmake --build --preset conan-debug
    ```

### Running Examples
The project includes several example binaries in the build output directory (e.g., `build/Debug/examples/`):
- `framework_quickstart_main`: Standard real-time engine entry point.
- `shadertoy_main`: Compute-based ShaderToy-style pipeline.
- `pbpt_cbox_roundtrip_main`: Scene XML conversion and validation.
- `framework_offline_cbox_main`: Real-time UI for PBPT offline rendering.

## Development Conventions

### Coding Style
- **Namespaces:** All code resides under the `rtr` namespace, further subdivided (e.g., `rtr::rhi`, `rtr::framework::core`).
- **Modern C++:** Use C++23 features. Prefer `std::unique_ptr` for ownership and `std::shared_ptr` only where shared ownership is strictly necessary.
- **Vulkan RAII:** Use `vk::raii` wrappers from `vulkan-hpp`. Avoid raw Vulkan handles unless interacting with external libraries.
- **Error Handling:** Use exceptions for fatal initialization errors and `RuntimeResult` for main loop failures.

### Shaders
- Shaders are written in **Slang** (`.slang`).
- They are compiled to SPIR-V during the build process and located in `build/Debug/shaders/compiled/`.
- Use `rhi::ShaderModule` to load them at runtime.

### Math
- Always use `pbpt::math` types (e.g., `pbpt::math::vec3`, `pbpt::math::mat4`).
- Be mindful of matrix layout: `pbpt::math` uses its own layout, but when packing into GPU uniform buffers (UBOs), use `rtr::system::render::pack_mat4_row_major` to ensure compatibility with standard Vulkan GLSL/HLSL layout expectations.

### Testing
- Tests are located in the `test/` directory.
- Use `ctest` to run all tests.
- **GPU Tests:** Tests requiring a Vulkan device are disabled by default. Set the environment variable `RTR_RUN_GPU_TESTS=1` to enable them.

## Key Files
- `conanfile.py`: Main dependency configuration.
- `CMakeLists.txt`: Project build configuration.
- `src/rtr/app/app_runtime.hpp`: Main engine entry point and loop logic.
- `src/rtr/system/render/renderer.hpp`: High-level Vulkan renderer.
- `src/rtr/framework/core/world.hpp`: Scene graph and entity management.
- `README.md`: Original project documentation with detailed setup steps.
