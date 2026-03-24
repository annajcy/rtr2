# Editor 总览

`src/rtr/editor/` 是建立在 runtime 和 render system 之上的 ImGui 工具层。

当前职责包括：

- editor host 与 panel 注册
- 场景检查与 gizmo 状态管理
- editor UI 与 runtime 之间的输入捕获仲裁
- `render/` 中的 editor 专用合成路径

下面文档里的 `render/` 子树并不是另一套独立场景渲染器。它消费 `src/rtr/system/render/` 产出的纯场景结果，再把 editor UI 合成到最终输出上。
