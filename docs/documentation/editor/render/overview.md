# Editor Render Overview

`src/rtr/editor/render/` contains the editor-specific composition layer that turns a pure scene pipeline output into the final editor frame.

Current split:

- `editor_output_backend.hpp`: renderer backend that owns the editor composition path
- `editor_imgui_pass.hpp`: the pass that draws ImGui and the scene view into the output target

The editor render layer does not replace the scene pipeline. It sits after `pipeline.final_output(...)` and consumes that image as an editor scene texture.
