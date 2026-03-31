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

## Build Documentation

```bash
uv run python scripts/docs/generate_source_tree_nav.py --check
doxygen Doxyfile
uv run mkdocs build --strict
```

For local preview:

```bash
uv run mkdocs serve
```

If you changed API-facing headers and already have a configured CMake build tree, you can also regenerate Doxygen with:

```bash
cmake --build --preset conan-debug --target rtr_docs
```

## Headless MCP

For HTTP-based headless scene authoring and PBPT offline rendering, see [Headless MCP](headless-mcp.md).
