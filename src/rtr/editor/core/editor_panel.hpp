#pragma once

#include <string_view>

#include "rtr/editor/core/editor_context.hpp"

namespace rtr::editor {

class IEditorPanel {
public:
    virtual ~IEditorPanel() = default;

    virtual std::string_view id() const = 0;
    virtual int order() const { return 0; }
    virtual bool visible() const { return true; }
    virtual void set_visible(bool /*visible*/) {}

    virtual void on_imgui(EditorContext& ctx) = 0;
};

} // namespace rtr::editor
