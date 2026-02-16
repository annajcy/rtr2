#pragma once

#include <memory>
#include <stdexcept>

#include "rtr/editor/editor_capture.hpp"
#include "rtr/editor/editor_host.hpp"
#include "rtr/editor/render/editor_overlay_pipeline.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

inline std::shared_ptr<spdlog::logger> editor_attach_logger() {
    return utils::get_logger("editor.attach");
}

inline std::unique_ptr<render::EditorOverlayPipeline> create_editor_pipeline(
    const system::render::PipelineRuntime& runtime,
    std::unique_ptr<system::render::IRenderPipeline> runtime_pipeline,
    const std::shared_ptr<EditorHost>& editor_host
) {
    if (!runtime_pipeline) {
        editor_attach_logger()->error("create_editor_pipeline failed: runtime_pipeline is null.");
        throw std::invalid_argument("create_editor_pipeline requires non-null runtime pipeline.");
    }
    if (!editor_host) {
        editor_attach_logger()->error("create_editor_pipeline failed: editor_host is null.");
        throw std::invalid_argument("create_editor_pipeline requires non-null editor_host.");
    }
    editor_attach_logger()->info("Editor overlay pipeline created.");
    return std::make_unique<render::EditorOverlayPipeline>(
        runtime,
        std::move(runtime_pipeline),
        editor_host
    );
}

inline void bind_input_capture_to_editor(
    system::input::InputSystem& input,
    IEditorInputCaptureSource& source
) {
    editor_attach_logger()->debug("Input capture hook bound to editor overlay pipeline.");
    input.set_is_intercept_capture([&source](bool is_mouse) {
        if (is_mouse) {
            return source.wants_imgui_capture_mouse();
        }
        return source.wants_imgui_capture_keyboard();
    });
}

} // namespace rtr::editor
