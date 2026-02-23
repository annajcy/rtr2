#pragma once

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "imgui.h"
#include "imgui_internal.h"

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class EditorHost final {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.host");
    }

    EditorContext m_context;
    std::vector<std::unique_ptr<IEditorPanel>> m_panels{};
    bool m_panels_dirty{false};
    bool m_default_layout_initialized{false};
    ImGuiDockNodeFlags m_dockspace_flags{ImGuiDockNodeFlags_PassthruCentralNode};

    void sort_panels_if_needed() {
        if (!m_panels_dirty) {
            return;
        }
        std::sort(
            m_panels.begin(),
            m_panels.end(),
            [](const std::unique_ptr<IEditorPanel>& lhs, const std::unique_ptr<IEditorPanel>& rhs) {
                if (lhs->order() != rhs->order()) {
                    return lhs->order() < rhs->order();
                }
                return lhs->id() < rhs->id();
            }
        );
        m_panels_dirty = false;
    }

    void set_panel_visible_if_exists(std::string_view panel_id, bool visible) {
        for (const auto& panel : m_panels) {
            if (!panel || panel->id() != panel_id) {
                continue;
            }
            panel->set_visible(visible);
            return;
        }
    }

    IEditorPanel* find_panel(std::string_view panel_id) {
        for (const auto& panel : m_panels) {
            if (panel && panel->id() == panel_id) {
                return panel.get();
            }
        }
        return nullptr;
    }

    const IEditorPanel* find_panel(std::string_view panel_id) const {
        for (const auto& panel : m_panels) {
            if (panel && panel->id() == panel_id) {
                return panel.get();
            }
        }
        return nullptr;
    }

    static std::string panel_menu_label(std::string_view panel_id) {
        if (panel_id == "scene_view") {
            return "Scene";
        }
        if (panel_id == "hierarchy") {
            return "Hierarchy";
        }
        if (panel_id == "inspector") {
            return "Inspector";
        }
        if (panel_id == "stats") {
            return "Stats";
        }
        if (panel_id == "logger") {
            return "Logger";
        }
        if (panel_id == "offline_render") {
            return "Offline Render";
        }
        return std::string(panel_id);
    }

    bool draw_window_menu() {
        bool reset_layout_requested = false;
        if (!ImGui::BeginMainMenuBar()) {
            return false;
        }

        if (ImGui::BeginMenu("Window")) {
            if (ImGui::MenuItem("Reset Layout")) {
                reset_layout_requested = true;
            }
            ImGui::Separator();

            sort_panels_if_needed();
            for (const auto& panel : m_panels) {
                if (!panel) {
                    continue;
                }
                bool visible = panel->visible();
                const std::string label = panel_menu_label(panel->id());
                if (ImGui::MenuItem(label.c_str(), nullptr, visible)) {
                    panel->set_visible(!visible);
                }
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
        return reset_layout_requested;
    }

    void apply_default_visibility(bool low_resolution_mode) {
        if (low_resolution_mode) {
            set_panel_visible_if_exists("logger", false);
            set_panel_visible_if_exists("inspector", false);
            set_panel_visible_if_exists("hierarchy", true);
            set_panel_visible_if_exists("scene_view", true);
            return;
        }

        set_panel_visible_if_exists("logger", true);
        set_panel_visible_if_exists("inspector", true);
        set_panel_visible_if_exists("hierarchy", true);
        set_panel_visible_if_exists("scene_view", true);
    }

    void ensure_default_layout(ImGuiID dockspace_id) {
        if (m_default_layout_initialized) {
            return;
        }

        ImGuiViewport* viewport = ImGui::GetMainViewport();
        if (viewport == nullptr) {
            return;
        }

        ImGui::DockBuilderRemoveNode(dockspace_id);
        ImGui::DockBuilderAddNode(dockspace_id, ImGuiDockNodeFlags_None);
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

        ImGuiID dock_id_main = dockspace_id;
        ImGuiID dock_id_right = ImGui::DockBuilderSplitNode(
            dock_id_main,
            ImGuiDir_Right,
            0.25f,
            nullptr,
            &dock_id_main
        );
        ImGuiID dock_id_left = ImGui::DockBuilderSplitNode(
            dock_id_main,
            ImGuiDir_Left,
            0.20f,
            nullptr,
            &dock_id_main
        );
        ImGuiID dock_id_bottom = ImGui::DockBuilderSplitNode(
            dock_id_main,
            ImGuiDir_Down,
            0.22f,
            nullptr,
            &dock_id_main
        );
        ImGuiID dock_id_stats = ImGui::DockBuilderSplitNode(
            dock_id_left,
            ImGuiDir_Down,
            0.42f,
            nullptr,
            &dock_id_left
        );

        ImGui::DockBuilderDockWindow("Hierarchy", dock_id_left);
        ImGui::DockBuilderDockWindow("Stats", dock_id_stats);
        ImGui::DockBuilderDockWindow("Inspector", dock_id_right);
        ImGui::DockBuilderDockWindow("Logger", dock_id_bottom);
        ImGui::DockBuilderDockWindow("Scene", dock_id_main);
        ImGui::DockBuilderDockWindow("Offline Render", dock_id_main);

        ImGui::DockBuilderFinish(dockspace_id);

        const bool low_resolution_mode =
            viewport->Size.x < 1100.0f ||
            viewport->Size.y < 700.0f;
        apply_default_visibility(low_resolution_mode);
        m_default_layout_initialized = true;
    }

public:
    explicit EditorHost(app::AppRuntime& runtime)
        : m_context(runtime.world(), runtime.resource_manager(), runtime.renderer(), runtime.input_system()) {}

    void reset_layout() {
        m_default_layout_initialized = false;
    }

    bool set_panel_visible(std::string_view panel_id, bool visible) {
        if (auto* panel = find_panel(panel_id); panel != nullptr) {
            panel->set_visible(visible);
            return true;
        }
        return false;
    }

    std::optional<bool> panel_visible(std::string_view panel_id) const {
        if (const auto* panel = find_panel(panel_id); panel != nullptr) {
            return panel->visible();
        }
        return std::nullopt;
    }

    EditorContext& context() {
        return m_context;
    }

    const EditorContext& context() const {
        return m_context;
    }

    void register_panel(std::unique_ptr<IEditorPanel> panel) {
        if (!panel) {
            logger()->error("register_panel failed: panel is null.");
            throw std::invalid_argument("EditorHost register_panel received null panel.");
        }
        const std::string incoming_id{panel->id()};
        for (const auto& existing : m_panels) {
            if (existing && existing->id() == incoming_id) {
                logger()->error("register_panel failed: duplicate panel id='{}'.", incoming_id);
                throw std::runtime_error("EditorHost duplicate panel id: " + incoming_id);
            }
        }
        const int incoming_order = panel->order();
        const bool incoming_visible = panel->visible();
        m_panels.emplace_back(std::move(panel));
        m_panels_dirty = true;
        logger()->debug(
            "Panel registered (id='{}', order={}, visible={}, panel_count={}).",
            incoming_id,
            incoming_order,
            incoming_visible,
            m_panels.size()
        );
    }

    template <typename TPanel, typename... TArgs>
    TPanel& emplace_panel(TArgs&&... args) {
        auto panel = std::make_unique<TPanel>(std::forward<TArgs>(args)...);
        TPanel* ptr = panel.get();
        register_panel(std::move(panel));
        return *ptr;
    }

    bool remove_panel(std::string_view panel_id) {
        const auto it = std::find_if(
            m_panels.begin(),
            m_panels.end(),
            [panel_id](const std::unique_ptr<IEditorPanel>& panel) {
                return panel && panel->id() == panel_id;
            }
        );
        if (it == m_panels.end()) {
            logger()->warn("remove_panel ignored: panel id='{}' not found.", panel_id);
            return false;
        }
        const std::string removed_id = std::string((*it)->id());
        m_panels.erase(it);
        logger()->debug(
            "Panel removed (id='{}', panel_count={}).",
            removed_id,
            m_panels.size()
        );
        return true;
    }

    std::size_t panel_count() const {
        return m_panels.size();
    }

    void begin_frame(const EditorFrameData& frame_data) {
        m_context.set_frame_data(frame_data);
        const auto previous_selection = m_context.selection();
        m_context.validate_selection();
        if (previous_selection.has_game_object() &&
            !m_context.selection().has_game_object()) {
            logger()->debug(
                "Selection cleared during begin_frame (scene_id={}, game_object_id={}).",
                previous_selection.scene_id,
                previous_selection.game_object_id
            );
        }
    }

    void draw_imgui() {
        bool reset_layout_requested = false;
        if (ImGui::GetCurrentContext() != nullptr) {
            reset_layout_requested = draw_window_menu();
        }

        if (ImGui::GetCurrentContext() != nullptr) {
            const ImGuiID dockspace_id =
                ImGui::DockSpaceOverViewport(0, ImGui::GetMainViewport(), m_dockspace_flags);
            if (reset_layout_requested) {
                reset_layout();
            }
            ensure_default_layout(dockspace_id);
        }

        sort_panels_if_needed();
        for (const auto& panel : m_panels) {
            if (panel && panel->visible()) {
                panel->on_imgui(m_context);
            }
        }
    }
};

} // namespace rtr::editor
