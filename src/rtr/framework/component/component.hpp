#pragma once

#include "rtr/framework/core/tick_context.hpp"

namespace rtr::framework::core {
class GameObject;
}

namespace rtr::framework::component {

class Component {
private:
    bool m_enabled{true};
    core::GameObject& m_owner;

public:
    explicit Component(core::GameObject& owner)
        : m_owner(owner) {}

    virtual ~Component() = default;
    Component(const Component&) = delete;
    Component& operator=(const Component&) = delete;
    Component(Component&&) = delete;
    Component& operator=(Component&&) = delete;

    bool enabled() const {
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }

    core::GameObject& owner() {
        return m_owner;
    }

    const core::GameObject& owner() const {
        return m_owner;
    }

    virtual void on_awake() {}
    virtual void on_fixed_update(const core::FixedTickContext& /*ctx*/) {}
    virtual void on_update(const core::FrameTickContext& /*ctx*/) {}
    virtual void on_late_update(const core::FrameTickContext& /*ctx*/) {}
    virtual void on_destroy() {}
};

} // namespace rtr::framework::component
