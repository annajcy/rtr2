#pragma once

#include <cstddef>
#include <memory>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <typeinfo>
#include <utility>
#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/scene_graph.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::core {

class GameObject {
private:
    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("framework.core.game_object"); }

    GameObjectId m_id{core::kInvalidGameObjectId};
    bool         m_components_destroyed{false};
    SceneGraph&  m_scene_graph;
    std::vector<std::unique_ptr<component::Component>> m_components{};

public:
    explicit GameObject(GameObjectId id, SceneGraph& scene_graph) : m_id(id), m_scene_graph(scene_graph) {}

    ~GameObject() {
        try {
            destroy_components();
        } catch (...) {
            logger()->error("destroy_components threw during GameObject destructor (game_object_id={}).", m_id);
        }
    }

    GameObject(const GameObject&) = delete;
    GameObject& operator=(const GameObject&) = delete;
    GameObject(GameObject&&) = delete;
    GameObject& operator=(GameObject&&) = delete;

    GameObjectId id() const { return m_id; }
    bool enabled() const { return node().is_enabled(); }

    void set_enabled(bool enabled) {
        m_scene_graph.set_enabled(m_id, enabled);
    }

    SceneGraph::NodeView node() {
        return m_scene_graph.node(m_id);
    }

    SceneGraph::ConstNodeView node() const {
        return m_scene_graph.node(m_id);
    }

    std::size_t component_count() const { return m_components.size(); }

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
        static_assert(std::is_constructible_v<TComponent, GameObject&, TArgs...>,
                      "Component type must be constructible with (GameObject&, ...).");
        if (has_component<TComponent>()) {
            logger()->warn("add_component rejected: duplicate component type '{}' on GameObject {}.",
                           typeid(TComponent).name(), m_id);
            throw std::runtime_error("GameObject already has this component type.");
        }
        auto component = std::make_unique<TComponent>(*this, std::forward<TArgs>(args)...);
        component->on_awake();
        TComponent* instance = component.get();
        m_components.emplace_back(std::move(component));
        logger()->debug("Component added (game_object_id={}, component_type='{}', component_count={})", m_id,
                        typeid(TComponent).name(), m_components.size());
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

    template <typename TComponent>
    TComponent& component_or_throw() {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
        auto* found = get_component<TComponent>();
        if (found == nullptr) {
            logger()->error("component_or_throw failed: GameObject {} missing component type '{}'.",
                            m_id, typeid(TComponent).name());
            throw std::runtime_error(std::string("GameObject missing required component: ") + typeid(TComponent).name());
        }
        return *found;
    }

    template <typename TComponent>
    const TComponent& component_or_throw() const {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
        const auto* found = get_component<TComponent>();
        if (found == nullptr) {
            logger()->error("component_or_throw failed: GameObject {} missing component type '{}'.",
                            m_id, typeid(TComponent).name());
            throw std::runtime_error(std::string("GameObject missing required component: ") + typeid(TComponent).name());
        }
        return *found;
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

}  // namespace rtr::framework::core
