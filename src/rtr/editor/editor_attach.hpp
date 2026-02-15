#pragma once

#include <memory>
#include <stdexcept>

#include "rtr/editor/editor_host.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

inline std::shared_ptr<spdlog::logger> editor_attach_logger() {
    return utils::get_logger("editor.attach");
}

inline system::render::IImGuiOverlayPipeline& require_imgui_overlay_pipeline(
    system::render::IRenderPipeline& pipeline
) {
    auto* overlay_pipeline = dynamic_cast<system::render::IImGuiOverlayPipeline*>(&pipeline);
    if (overlay_pipeline == nullptr) {
        editor_attach_logger()->error(
            "require_imgui_overlay_pipeline failed: pipeline does not implement IImGuiOverlayPipeline."
        );
        throw std::runtime_error("Render pipeline does not implement IImGuiOverlayPipeline.");
    }
    return *overlay_pipeline;
}

inline void attach_editor_host(
    system::render::IRenderPipeline& pipeline,
    const std::shared_ptr<EditorHost>& editor_host
) {
    if (!editor_host) {
        editor_attach_logger()->error("attach_editor_host failed: editor_host is null.");
        throw std::invalid_argument("attach_editor_host requires non-null editor_host.");
    }
    require_imgui_overlay_pipeline(pipeline).set_imgui_overlay(editor_host);
    editor_attach_logger()->info("EditorHost attached to ImGui overlay pipeline.");
}

inline void detach_editor_host(system::render::IRenderPipeline& pipeline) {
    require_imgui_overlay_pipeline(pipeline).clear_imgui_overlay();
    editor_attach_logger()->debug("EditorHost detached from ImGui overlay pipeline.");
}

inline void bind_input_capture_to_pipeline(
    system::input::InputSystem& input,
    system::render::IRenderPipeline& pipeline
) {
    editor_attach_logger()->debug("Input capture hook bound to ImGui overlay pipeline.");
    input.set_is_intercept_capture([&pipeline](bool is_mouse) {
        auto* overlay_pipeline = dynamic_cast<system::render::IImGuiOverlayPipeline*>(&pipeline);
        if (overlay_pipeline == nullptr) {
            return false;
        }
        if (is_mouse) {
            return overlay_pipeline->wants_imgui_capture_mouse();
        }
        return overlay_pipeline->wants_imgui_capture_keyboard();
    });
}

} // namespace rtr::editor
