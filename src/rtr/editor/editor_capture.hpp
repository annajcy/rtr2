#pragma once

namespace rtr::editor {

class IEditorInputCaptureSource {
public:
    virtual ~IEditorInputCaptureSource() = default;
    virtual bool wants_imgui_capture_mouse() const = 0;
    virtual bool wants_imgui_capture_keyboard() const = 0;
};

} // namespace rtr::editor

