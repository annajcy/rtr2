#pragma once

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rtr/editor/editor_panel.hpp"
#include "rtr/system/render/render_pass.hpp"

namespace rtr::editor {

class EditorHost final : public system::render::IImGuiOverlay {
private:
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
    }

    EditorContext& context() {
        return m_context;
    }

    const EditorContext& context() const {
        return m_context;
    }

    void register_panel(std::unique_ptr<IEditorPanel> panel) {
        if (!panel) {
            throw std::invalid_argument("EditorHost register_panel received null panel.");
        }
        const std::string incoming_id{panel->id()};
        for (const auto& existing : m_panels) {
            if (existing && existing->id() == incoming_id) {
                throw std::runtime_error("EditorHost duplicate panel id: " + incoming_id);
            }
        }
        m_panels.emplace_back(std::move(panel));
        m_panels_dirty = true;
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
            return false;
        }
        m_panels.erase(it);
        return true;
    }

    std::size_t panel_count() const {
        return m_panels.size();
    }

    void begin_frame(const EditorFrameData& frame_data) {
        m_context.set_frame_data(frame_data);
        m_context.validate_selection();
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
