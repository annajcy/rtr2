#pragma once

#include <stdexcept>

#include "imgui.h"

#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class StatsPanel final : public IEditorPanel {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.panel.stats");
    }

    bool m_visible{true};
    int m_order{300};

public:
    std::string_view id() const override {
        return "stats";
    }

    int order() const override {
        return m_order;
    }

    bool visible() const override {
        return m_visible;
    }

    void set_visible(bool visible) override {
        m_visible = visible;
    }

    void on_imgui(EditorContext& ctx) override {
        if (!m_visible) {
            return;
        }

        if (!ctx.is_bound()) {
            logger()->error("StatsPanel on_imgui failed: EditorContext is not fully bound.");
            throw std::runtime_error("StatsPanel requires bound EditorContext.");
        }

        if (!ImGui::Begin("Stats", &m_visible)) {
            ImGui::End();
            return;
        }

        const auto& frame_data = ctx.frame_data();
        const auto& io = ImGui::GetIO();

        ImGui::Text("FPS: %.1f", io.Framerate);
        ImGui::Text("Frame Serial: %llu", static_cast<unsigned long long>(frame_data.frame_serial));
        ImGui::Text("Delta: %.4f ms", frame_data.delta_seconds * 1000.0);
        ImGui::Text("Paused: %s", frame_data.paused ? "true" : "false");

        const auto& world = ctx.world();
        ImGui::Separator();
        ImGui::Text("Scene Count: %zu", world.scene_count());

        if (const auto* active_scene = world.active_scene(); active_scene != nullptr) {
            ImGui::Text("Active Scene: %s (%llu)", active_scene->name().c_str(), static_cast<unsigned long long>(active_scene->id()));
            ImGui::Text("GameObjects: %zu", active_scene->game_object_count());
        } else {
            ImGui::TextDisabled("Active Scene: none");
        }

        ImGui::End();
    }
};

} // namespace rtr::editor
