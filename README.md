# RTR2 - Real-Time Renderer

Compared to RTR, RTR2 focuses on modern rendering API support (Vulkan) and modern Shading Language (Slang). It also explores intergrating Machine Learning techniques into real-time rendering.

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
SHA=$(git rev-parse --short HEAD)
PBPT_VER="0.1.0-dev.${SHA}"

uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o "&:with_pbpt=True" -o "&:pbpt_version=${PBPT_VER}" --build=missing
```

```bash
# quick local example
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o '&:with_pbpt=True' -o '&:pbpt_version=0.1.0-dev.local' --build=missing
```

Create a local consumable package:
```bash
# Use a real suffix. Example below uses git short SHA.
SHA=$(git rev-parse --short HEAD)
PBPT_VER="0.1.0-dev.${SHA}"
RTR_VER="0.1.0-dev.${SHA}"

# 1) create pbpt package in local cache
uv run conan create external/pbpt --version ${PBPT_VER} -s build_type=Debug -s compiler.cppstd=23 --build=missing

# 2) create rtr package in local cache (Conan 2 scoped option syntax)
uv run conan create . --name=rtr --version ${RTR_VER} -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o "&:with_pbpt=True" -o "&:pbpt_version=${PBPT_VER}" --build=missing
```

Quick local example:
```bash
uv run conan create external/pbpt --version 0.1.0-dev.local -s build_type=Debug -s compiler.cppstd=23 --build=missing
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o '&:with_pbpt=True' -o '&:pbpt_version=0.1.0-dev.local' --build=missing
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
uv run conan install . -pr=profiles/rtr2 -pr=rtr2-local -s build_type=Debug -s compiler.cppstd=23 -o '&:with_pbpt=True' -o '&:pbpt_version=0.1.0-dev.<sha>' --build=missing
```

### Troubleshooting dependency conflicts

If Conan resolves an older cached `pbpt` (for example `pbpt/0.1.0`) and reports conflicts like `imgui` vs `imgui-docking`, remove the old cache entry and retry with scoped `pbpt_version`:

```bash
conan remove 'pbpt/0.1.0*' -c
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o '&:with_pbpt=True' -o '&:pbpt_version=0.1.0-dev.<sha>' --build=missing
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
target_link_libraries(your_app PRIVATE rtr::rtr rtr::framework_integration)
```

PBPT runtime can be disabled while still exporting the same targets:
```bash
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 -o '&:with_pbpt=False' --build=missing
```
When `with_pbpt=False`, `rtr::framework_integration` links a stub implementation:
- target name remains `rtr::framework_integration`
- `PbptOfflineRenderService::start(...)` returns `false`
- no `pbpt`/`OpenEXR`/`embree`/`TBB` package resolution is required

Configure and build with CMake presets:
```bash
cmake --list-presets
cmake --preset conan-debug
cmake --build --preset conan-debug
```
If the preset name differs, run `cmake --list-presets` to check the available names.

# PBPT integration

Initialize the PBPT submodule:
```bash
git submodule update --init --recursive
```

`PbptMesh` and `PbptLight` use spectrum points (`lambda_nm`, `value`) and
serialize to Mitsuba-style XML:
- BSDF reflectance: `<spectrum name="reflectance" value="..."/>`
- Area light radiance: `<spectrum name="radiance" value="..."/>`

Default spectrum points:
- Reflectance: `400:0.7, 500:0.7, 600:0.7, 700:0.7`
- Radiance: `400:1.0, 500:1.0, 600:1.0, 700:1.0`

To manually validate Cornell Box rendering in PBPT:
```bash
cd external/pbpt
# follow PBPT's own dependency/build steps
# then run a cbox example that loads:
# asset/scene/cbox/cbox.xml
```
