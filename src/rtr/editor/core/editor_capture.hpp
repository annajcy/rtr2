#pragma once

#include "rtr/system/input/input_system.hpp"

namespace rtr::editor {

class IEditorInputCaptureSource {
public:
    virtual ~IEditorInputCaptureSource()              = default;
    virtual bool wants_imgui_capture_mouse() const    = 0;
    virtual bool wants_imgui_capture_keyboard() const = 0;
};

inline void bind_input_capture_to_editor(system::input::InputSystem& input, IEditorInputCaptureSource& source) {
    input.set_is_intercept_capture([&source](bool is_mouse) {
        if (is_mouse) {
            return source.wants_imgui_capture_mouse();
        }
        return source.wants_imgui_capture_keyboard();
    });
}

}  // namespace rtr::editor
