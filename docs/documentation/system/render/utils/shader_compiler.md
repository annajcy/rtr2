# Shader Compiler

`src/rtr/system/render/utils/shader_compiler.hpp` wraps file-based Slang compilation into a small result object usable from runtime code.

It provides:

- `SlangFileCompileResult`
- `compile_slang_file_to_spirv(...)`

The helper performs:

- source-file existence and timestamp checks
- Slang session/request setup
- stage validation for the requested entry point kind
- SPIR-V extraction into an in-memory byte buffer

This utility is intentionally runtime-oriented. It is used for hot-reload and editable pipelines, not as the project's full offline shader build pipeline.
