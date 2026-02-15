#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtr/editor/editor_panel.hpp"
#include "rtr/system/render/render_pass.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class EditorHost final : public system::render::IImGuiOverlay {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.host");
    }

    EditorContext m_context{};
    std::vector<std::unique_ptr<IEditorPanel>> m_panels{};
    bool m_panels_dirty{false};

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

public:
    EditorHost() = default;

    void bind_runtime(
        framework::core::World* world,
        resource::ResourceManager* resources,
        system::render::Renderer* renderer,
        system::input::InputSystem* input
    ) {
        m_context.bind_runtime(world, resources, renderer, input);
        logger()->info(
            "Editor runtime bound (world={}, resources={}, renderer={}, input={}).",
            world != nullptr,
            resources != nullptr,
            renderer != nullptr,
            input != nullptr
        );
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
        sort_panels_if_needed();
        for (const auto& panel : m_panels) {
            if (panel && panel->visible()) {
                panel->on_frame(m_context);
            }
        }
    }

    void draw_imgui() override {
        sort_panels_if_needed();
        for (const auto& panel : m_panels) {
            if (panel && panel->visible()) {
                panel->on_imgui(m_context);
            }
        }
    }
};

} // namespace rtr::editor
