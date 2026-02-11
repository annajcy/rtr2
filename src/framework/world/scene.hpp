#pragma once

#include <string>
#include <utility>

#include "framework/core/types.hpp"

namespace rtr::framework::world {

class Scene {
private:
    core::SceneId m_id{core::kInvalidSceneId};
    std::string m_name{"Scene"};
    bool m_enabled{true};

public:
    explicit Scene(
        core::SceneId id = core::kInvalidSceneId,
        std::string name = "Scene"
    )
        : m_id(id), m_name(std::move(name)) {}

    core::SceneId id() const {
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
