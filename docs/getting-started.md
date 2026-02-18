# Getting Started

## Prerequisites

- CMake 3.27+
- Conan 2+
- Ninja
- Python with `uv`

## Build (Debug)

```bash
uv sync
uv run conan profile detect --force
cd conan_recipe && uv run python build_conan_recipes.py -d . -b Debug -v && cd ..
uv run conan install . -pr=profiles/rtr2 -s build_type=Debug -s compiler.cppstd=23 --build=missing
cmake --preset conan-debug
cmake --build --preset conan-debug
```

## Run unit tests

```bash
ctest --test-dir build/Debug -C Debug -LE integration
```

## Run GPU integration tests

```bash
RTR_RUN_GPU_TESTS=1 ctest --test-dir build/Debug -C Debug -L integration
```
