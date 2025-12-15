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

```bash
cd conan_recipe
python build_conan_recipes.py -d . -v
cd ..
```

```bash
conan build . -s build_type=Debug -s compiler.cppstd=17 -c tools.system.package_manager:mode=install -c tools.system.package_manager:sudo=True --build=missing
```
