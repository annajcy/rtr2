# Editor Overview

`src/rtr/editor/` contains the ImGui-driven tooling layer on top of the runtime and render system.

Current responsibilities:

- editor host and panel registration
- scene inspection and gizmo state management
- input capture arbitration between the editor UI and the runtime
- editor-specific render composition in `render/`

The `render/` subtree documented below is not a separate scene renderer. It consumes the pure scene output produced by `src/rtr/system/render/` and composites editor UI on top of it.
