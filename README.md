# RTR2 - Real-Time Renderer

Compared to RTR, RTR2 focuses on modern rendering API support (Vulkan) and modern Shading Language (Slang). It also explores intergrating Machine Learning techniques into real-time rendering.

# How to build

### Install UV

```bash
conda install conda-forge::uv
conda install conda-forge::conan
```
```bash
uv sync 
```

if using macos/linux
```bash
source .venv/bin/activate
```

if using windows
```powershell
.venv\Scripts\activate
```

### Conan Build

```bash
conan profile detect --force
```

Before running `conan install`, register the local `slang` recipe:
```bash
cd conan_recipe
python build_conan_recipes.py -d . -v
cd ..
```

Use the project profile (cross-platform, Ninja):
```bash
conan install . -pr=profiles/rtr2 -s build_type=Debug --build=missing
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
conan install . -pr=profiles/rtr2 -pr=rtr2-local -s build_type=Debug --build=missing
```

Configure and build with CMake presets:
```bash
cmake --list-presets
cmake --preset conan-debug
cmake --build --preset conan-debug
```
If the preset name differs, run `cmake --list-presets` to check the available names.
