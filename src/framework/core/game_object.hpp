#pragma once

#include <memory>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "framework/component/component.hpp"
#include "framework/core/tick_context.hpp"
#include "framework/core/types.hpp"

namespace rtr::framework::core {

class GameObject {
private:
    GameObjectId m_id{core::kInvalidGameObjectId};
    std::string m_name{"GameObject"};
    bool m_enabled{true};
    std::vector<std::unique_ptr<component::Component>> m_components{};

public:
    explicit GameObject(
        GameObjectId id = core::kInvalidGameObjectId,
        std::string name = "GameObject"
    )
        : m_id(id), m_name(std::move(name)) {}

    ~GameObject() {
        for (const auto& component : m_components) {
            if (component) {
                component->on_destroy();
            }
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
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }

    template <typename TComponent, typename... TArgs>
    TComponent& add_component(TArgs&&... args) {
        static_assert(std::is_base_of_v<component::Component, TComponent>);
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

    void fixed_tick(const FixedTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        for (const auto& component : m_components) {
            if (component && component->enabled()) {
                component->on_fixed_update(ctx);
            }
        }
    }

    void tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
            return;
        }
        for (const auto& component : m_components) {
            if (component && component->enabled()) {
                component->on_update(ctx);
            }
        }
    }

    void late_tick(const FrameTickContext& ctx) {
        if (!m_enabled) {
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
