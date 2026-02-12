#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "framework/component/component.hpp"
#include "framework/core/scene_graph.hpp"
#include "framework/core/tick_context.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {

class GameObject {
private:
    GameObjectId m_id{core::kInvalidGameObjectId};
    std::string m_name{"GameObject"};
    bool m_components_destroyed{false};
    SceneGraph* m_scene_graph{nullptr};
    std::vector<std::unique_ptr<component::Component>> m_components{};

    void bind_scene_graph(SceneGraph* scene_graph) {
        m_scene_graph = scene_graph;
    }

    friend class Scene;

public:
    explicit GameObject(
        GameObjectId id = core::kInvalidGameObjectId,
        std::string name = "GameObject"
    )
        : m_id(id), m_name(std::move(name)) {}

    ~GameObject() {
        try {
            destroy_components();
        } catch (...) {
            // Destructor must not throw.
        }
    }

    GameObject(const GameObject&) = delete;
    GameObject& operator=(const GameObject&) = delete;
    GameObject(GameObject&&) noexcept = default;
    GameObject& operator=(GameObject&&) noexcept = default;

    GameObjectId id() const {
        return m_id;
    }

    const std::string& name() const {
        return m_name;
    }

    void set_name(std::string name) {
        m_name = std::move(name);
    }

    bool enabled() const {
        return node().is_enabled();
    }

    void set_enabled(bool enabled) {
        if (m_scene_graph == nullptr) {
            throw std::runtime_error("GameObject is not attached to a SceneGraph.");
        }
        m_scene_graph->set_enabled(m_id, enabled);
    }

    bool has_scene_graph() const {
        return m_scene_graph != nullptr;
    }

    SceneGraph::NodeView node() {
        if (m_scene_graph == nullptr) {
            throw std::runtime_error("GameObject is not attached to a SceneGraph.");
        }
        return m_scene_graph->node(m_id);
    }

    SceneGraph::ConstNodeView node() const {
        if (m_scene_graph == nullptr) {
            throw std::runtime_error("GameObject is not attached to a SceneGraph.");
        }
        return m_scene_graph->node(m_id);
    }

    std::size_t component_count() const {
        return m_components.size();
    }

    void destroy_components() {
        if (m_components_destroyed) {
            return;
        }
        for (auto& component : m_components) {
            if (component) {
                component->on_destroy();
                component.reset();
            }
        }
        m_components.clear();
        m_components_destroyed = true;
    }

    template <typename TComponent, typename... TArgs>
    TComponent& add_component(TArgs&&... args) {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
        if (has_component<TComponent>()) {
            throw std::runtime_error("GameObject already has this component type.");
        }
        auto component = std::make_unique<TComponent>(std::forward<TArgs>(args)...);
        component->bind_owner(this);
        component->on_awake();
        TComponent* instance = component.get();
        m_components.emplace_back(std::move(component));
        return *instance;
    }

    template <typename TComponent>
    TComponent* get_component() {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
        for (const auto& component : m_components) {
            if (auto* found = dynamic_cast<TComponent*>(component.get()); found != nullptr) {
                return found;
            }
        }
        return nullptr;
    }

    template <typename TComponent>
    const TComponent* get_component() const {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
        for (const auto& component : m_components) {
            if (auto* found = dynamic_cast<const TComponent*>(component.get()); found != nullptr) {
                return found;
            }
        }
        return nullptr;
    }

    template <typename TComponent>
    bool has_component() const {
        return get_component<TComponent>() != nullptr;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        if (!enabled()) {
            return;
        }
        for (const auto& component : m_components) {
            if (component && component->enabled()) {
                component->on_fixed_update(ctx);
            }
        }
    }

    void tick(const FrameTickContext& ctx) {
        if (!enabled()) {
            return;
        }
        for (const auto& component : m_components) {
            if (component && component->enabled()) {
                component->on_update(ctx);
            }
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        if (!enabled()) {
            return;
        }
        for (const auto& component : m_components) {
            if (component && component->enabled()) {
                component->on_late_update(ctx);
            }
        }
    }
};

} // namespace rtr::framework::core
