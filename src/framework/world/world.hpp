#pragma once

#include "framework/core/types.hpp"

namespace rtr::framework::world {

class World {
private:
    core::WorldId m_id{core::kInvalidWorldId};
    bool m_enabled{true};

public:
    explicit World(core::WorldId id = core::kInvalidWorldId)
        : m_id(id) {}

    core::WorldId id() const {
        return m_id;
    }

    bool enabled() const {
        return m_enabled;
    }

    void set_enabled(bool enabled) {
        m_enabled = enabled;
    }
};

} // namespace rtr::framework::world
