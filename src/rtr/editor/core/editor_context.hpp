#pragma once

#include "rtr/editor/core/editor_types.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/renderer.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor {

class EditorContext {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("editor.context");
    }

    framework::core::World& m_world;
    resource::ResourceManager& m_resources;
    system::render::Renderer& m_renderer;
    system::input::InputSystem& m_input;
    EditorFrameData m_frame_data{};
    EditorSelection m_selection{};
    EditorServices m_services{};

public:
    framework::core::World& world() {
        return m_world;
    }

    const framework::core::World& world() const {
        return m_world;
    }

    resource::ResourceManager& resources() {
        return m_resources;
    }

    const resource::ResourceManager& resources() const {
        return m_resources;
    }

    system::render::Renderer& renderer() {
        return m_renderer;
    }

    const system::render::Renderer& renderer() const {
        return m_renderer;
    }

    system::input::InputSystem& input() {
        return m_input;
    }

    const system::input::InputSystem& input() const {
        return m_input;
    }

    EditorContext(
        framework::core::World& world,
        resource::ResourceManager& resources,
        system::render::Renderer& renderer,
        system::input::InputSystem& input
    )
        : m_world(world), m_resources(resources), m_renderer(renderer), m_input(input) {}

    void set_frame_data(const EditorFrameData& frame_data) {
        m_frame_data = frame_data;
    }

    const EditorFrameData& frame_data() const {
        return m_frame_data;
    }

    EditorSelection& selection() {
        return m_selection;
    }

    const EditorSelection& selection() const {
        return m_selection;
    }

    void set_selection(
        framework::core::SceneId scene_id,
        framework::core::GameObjectId game_object_id
    ) {
        if (m_selection.scene_id == scene_id &&
            m_selection.game_object_id == game_object_id) {
            return;
        }
        logger()->debug(
            "Selection changed (old_scene_id={}, old_game_object_id={}, new_scene_id={}, new_game_object_id={}).",
            m_selection.scene_id,
            m_selection.game_object_id,
            scene_id,
            game_object_id
        );
        m_selection.scene_id = scene_id;
        m_selection.game_object_id = game_object_id;
    }

    void clear_selection() {
        if (!m_selection.has_game_object()) {
            return;
        }
        logger()->debug(
            "Selection cleared (scene_id={}, game_object_id={}).",
            m_selection.scene_id,
            m_selection.game_object_id
        );
        m_selection.clear();
    }

    EditorServices& services() {
        return m_services;
    }

    const EditorServices& services() const {
        return m_services;
    }

    void validate_selection() {
        if (!m_selection.has_game_object()) {
            return;
        }

        const auto scene_id = m_selection.scene_id;
        const auto game_object_id = m_selection.game_object_id;
        auto* scene = m_world.find_scene(scene_id);
        if (scene == nullptr || !scene->has_game_object(m_selection.game_object_id)) {
            m_selection.clear();
            logger()->debug(
                "Selection invalidated and cleared (scene_id={}, game_object_id={}, scene_exists={}).",
                scene_id,
                game_object_id,
                scene != nullptr
            );
        }
    }
};

} // namespace rtr::editor
