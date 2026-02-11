#pragma once

#include <string>
#include <utility>

#include "framework/core/types.hpp"

namespace rtr::framework::world {

class GameObject {
private:
    core::GameObjectId m_id{core::kInvalidGameObjectId};
    std::string m_name{"GameObject"};
    bool m_enabled{true};

public:
    explicit GameObject(
        core::GameObjectId id = core::kInvalidGameObjectId,
        std::string name = "GameObject"
    )
        : m_id(id), m_name(std::move(name)) {}

    core::GameObjectId id() const {
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
};

} // namespace rtr::framework::world
