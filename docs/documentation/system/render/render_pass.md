# Render Pass

`src/rtr/system/render/render_pass.hpp` provides the minimal reusable base for concrete passes such as `ForwardPass`, `PresentPass`, and `EditorImGuiPass`.

The template intentionally stays small:

- `execute(...)` is the public entry
- `validate(...)` checks pass-specific resources
- `do_execute(...)` records the actual Vulkan commands

The helper methods `require(...)` and `require_valid_extent(...)` keep pass-level precondition checks consistent without forcing a larger framework.

This file does not own pipelines, frame scheduling, or synchronization. It is only a light structure for "validate resources, then record commands".
