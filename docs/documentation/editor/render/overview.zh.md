# Editor Render 总览

`src/rtr/editor/render/` 是 editor 专用的合成层，它把纯场景 pipeline 的输出变成最终 editor 帧。

当前拆分是：

- `editor_output_backend.hpp`：拥有 editor 合成路径的 renderer backend
- `editor_imgui_pass.hpp`：把 ImGui 和 scene view 画到输出目标上的 pass

editor render 层并不替代场景 pipeline。它位于 `pipeline.final_output(...)` 之后，把那张图像当成 editor scene texture 来消费。
