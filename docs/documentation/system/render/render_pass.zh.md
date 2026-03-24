# Render Pass

`src/rtr/system/render/render_pass.hpp` 为 `ForwardPass`、`PresentPass`、`EditorImGuiPass` 这类具体 pass 提供了一个极简可复用基类。

这个模板故意保持很小：

- `execute(...)` 是公共入口
- `validate(...)` 检查 pass 自己依赖的资源
- `do_execute(...)` 录制实际的 Vulkan 命令

辅助函数 `require(...)` 和 `require_valid_extent(...)` 用于统一前置条件检查，而不会引入更重的 pass framework。

这个文件不拥有 pipeline、frame scheduling 或同步逻辑。它只负责“先校验资源，再录命令”。
