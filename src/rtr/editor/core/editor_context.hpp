#pragma once

#include <stdexcept>

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

    framework::core::World* m_world{};
    resource::ResourceManager* m_resources{};
    system::render::Renderer* m_renderer{};
    system::input::InputSystem* m_input{};
    EditorFrameData m_frame_data{};
    EditorSelection m_selection{};
    EditorServices m_services{};

public:
    void bind_runtime(
        framework::core::World* world,
        resource::ResourceManager* resources,
        system::render::Renderer* renderer,
        system::input::InputSystem* input
    ) {
        m_world = world;
        m_resources = resources;
        m_renderer = renderer;
        m_input = input;
        logger()->debug(
            "Runtime dependency bound (world={}, resources={}, renderer={}, input={}).",
            world != nullptr,
            resources != nullptr,
            renderer != nullptr,
            input != nullptr
        );
    }

    bool is_bound() const {
        return m_world != nullptr &&
            m_resources != nullptr &&
            m_renderer != nullptr &&
            m_input != nullptr;
    }

    framework::core::World& world() {
        if (m_world == nullptr) {
            logger()->error("world() failed: world is not bound.");
            throw std::runtime_error("EditorContext world is not bound.");
        }
        return *m_world;
    }

    const framework::core::World& world() const {
        if (m_world == nullptr) {
            logger()->error("world() const failed: world is not bound.");
            throw std::runtime_error("EditorContext world is not bound.");
        }
        return *m_world;
    }

    resource::ResourceManager& resources() {
        if (m_resources == nullptr) {
            logger()->error("resources() failed: resources are not bound.");
            throw std::runtime_error("EditorContext resources are not bound.");
        }
        return *m_resources;
    }

    const resource::ResourceManager& resources() const {
        if (m_resources == nullptr) {
            logger()->error("resources() const failed: resources are not bound.");
            throw std::runtime_error("EditorContext resources are not bound.");
        }
        return *m_resources;
    }

    system::render::Renderer& renderer() {
        if (m_renderer == nullptr) {
            logger()->error("renderer() failed: renderer is not bound.");
            throw std::runtime_error("EditorContext renderer is not bound.");
        }
        return *m_renderer;
    }

    const system::render::Renderer& renderer() const {
        if (m_renderer == nullptr) {
            logger()->error("renderer() const failed: renderer is not bound.");
            throw std::runtime_error("EditorContext renderer is not bound.");
        }
        return *m_renderer;
    }

    system::input::InputSystem& input() {
        if (m_input == nullptr) {
            logger()->error("input() failed: input is not bound.");
            throw std::runtime_error("EditorContext input is not bound.");
        }
        return *m_input;
    }

    const system::input::InputSystem& input() const {
        if (m_input == nullptr) {
            logger()->error("input() const failed: input is not bound.");
            throw std::runtime_error("EditorContext input is not bound.");
        }
        return *m_input;
    }

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
        if (!m_selection.has_game_object() || m_world == nullptr) {
            return;
        }

        const auto scene_id = m_selection.scene_id;
        const auto game_object_id = m_selection.game_object_id;
        auto* scene = m_world->find_scene(scene_id);
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
