# RTR2 - Real-Time Renderer

Compared to RTR, RTR2 focuses on modern rendering API support (Vulkan) and modern Shading Language (Slang). It also explores intergrating Machine Learning techniques into real-time rendering.
Math stack: `pbpt::math` is used end-to-end, including `pbpt::math::Quat` for quaternions.

# How to build

### Install UV

```bash
conda install conda-forge::uv
```
```bash
uv sync 
```

`conan` is installed into the project virtual environment by `uv sync`.

Use `uv run` to execute Python/Conan commands; no manual environment activation is needed.

### Conan Build

```bash
uv run conan profile detect --force
```

Before running `conan install`, register the local `slang` recipe:
```bash
cd conan_recipe
uv run python build_conan_recipes.py -d . -b Debug -v
cd ..
```

Use the project profile (cross-platform, Ninja):
```bash
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
```

The build always uses vendored `external/pbpt` in this repository.

Create a local consumable package:
```bash
SHA=$(git rev-parse --short HEAD)
RTR_VER="0.1.0-dev.${SHA}"
uv run conan create . --name=rtr --version ${RTR_VER} -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
```

Windows note: if Conan cannot find your Visual Studio installation, set a
user-only profile and layer it on top of the project profile.
Run the helper script to create `~/.conan2/profiles/rtr2-local`:
```powershell
pwsh -ExecutionPolicy Bypass -File script\setup_conan_profile.ps1
```
If auto-detect fails, pass the VS path:
```powershell
pwsh -ExecutionPolicy Bypass -File script\setup_conan_profile.ps1 -VsPath "C:\Program Files\Microsoft Visual Studio\18\Community"
```

Then run:
```bash
uv run conan install . -pr=profiles/rtr2 -pr=rtr2-local -s build_type=Debug -s compiler.cppstd=23 --build=missing
```

### Troubleshooting dependency conflicts

If Conan cache contains stale `rtr` binaries after option/profile changes, clear cached `rtr` packages and retry:

```bash
conan remove 'rtr/*' -c
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
```

Do not run `<sha>` literally in shell variable assignments; it will be parsed as redirection in `zsh`.

### Downstream consumption

```python
# conanfile.py
def requirements(self):
    self.requires("rtr/0.1.0-dev.<sha>")
```

```cmake
find_package(rtr CONFIG REQUIRED)
target_link_libraries(your_app PRIVATE rtr::rtr)
```

Configure and build with CMake presets:
```bash
cmake --list-presets
cmake --preset conan-debug
cmake --build --preset conan-debug
```
If the preset name differs, run `cmake --list-presets` to check the available names.

## Examples

`build/Debug` below is an example output directory. Use the build directory produced by your active CMake preset.

### `framework_quickstart_main`
Purpose: Recommended first real-time example using `AppRuntime + ForwardPipeline`, scene setup, free-look camera, and ImGui controls.

Build command:
```bash
cmake --build build/Debug --target framework_quickstart_main
```

Run command:
```bash
./build/Debug/examples/framework_quickstart_main
```

### `shadertoy_main`
Purpose: Runs the compute-to-present ShaderToy-style pipeline with an ImGui overlay.

Build command:
```bash
cmake --build build/Debug --target shadertoy_main
```

Run command:
```bash
./build/Debug/examples/shadertoy_main
```

### `pbpt_cbox_roundtrip_main`
Purpose: Imports a PBPT scene XML into RTR structures and writes a roundtrip XML output.
Prerequisite: PBPT scene resources under `assets/pbpt_scene/cbox` must exist.

Build command:
```bash
cmake --build build/Debug --target pbpt_cbox_roundtrip_main
```

Run command:
```bash
./build/Debug/examples/pbpt_cbox_roundtrip_main pbpt_scene/cbox cbox.xml /tmp/cbox_rtr_roundtrip.xml
```

### `framework_offline_cbox_main`
Purpose: Loads the cbox scene in RTR and exposes PBPT offline render controls via ImGui.
Prerequisite: PBPT runtime is always enabled in this repository build.

Build command:
```bash
cmake --build build/Debug --target framework_offline_cbox_main
```

Run command:
```bash
./build/Debug/examples/framework_offline_cbox_main
```

# PBPT integration

Initialize the PBPT submodule:
```bash
git submodule update --init --recursive
```

`PbptMesh` uses `MeshRenderer.base_color` (`rgb`) as the only reflectance source,
and `PbptLight` uses spectrum points (`lambda_nm`, `value`) for area light radiance.
Serialization is Mitsuba-style XML:
- BSDF reflectance: `<rgb name="reflectance" value="r g b"/>`
- Area light radiance: `<spectrum name="radiance" value="..."/>`

Default spectrum points:
- Radiance: `400:1.0, 500:1.0, 600:1.0, 700:1.0`

To manually validate Cornell Box rendering in PBPT:
```bash
cd external/pbpt
# follow PBPT's own dependency/build steps
# then run a cbox example that loads:
# asset/scene/cbox/cbox.xml
```
