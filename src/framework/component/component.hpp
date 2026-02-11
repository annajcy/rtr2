#pragma once

#include "framework/core/tick_context.hpp"

namespace rtr::framework::world {
class GameObject;
}

namespace rtr::framework::component {

class Component {
private:
    bool m_enabled{true};
    world::GameObject* m_owner{nullptr};

protected:
    void bind_owner(world::GameObject* owner) {
        m_owner = owner;
    }

    friend class rtr::framework::world::GameObject;

public:
    virtual ~Component() = default;

    bool enabled() const {
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }

    world::GameObject* owner() {
        return m_owner;
    }

    const world::GameObject* owner() const {
        return m_owner;
    }

    virtual void on_awake() {}
    virtual void on_fixed_update(const core::FixedTickContext& /*ctx*/) {}
    virtual void on_update(const core::FrameTickContext& /*ctx*/) {}
    virtual void on_late_update(const core::FrameTickContext& /*ctx*/) {}
    virtual void on_destroy() {}
};

} // namespace rtr::framework::component
