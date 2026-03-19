#pragma once

#include <pbpt/math/math.h>

#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/system/physics/cloth/cloth_state.hpp"

namespace rtr::system::physics {

struct ClothParams {
    pbpt::math::Float default_vertex_mass{1.0f};
};

struct ClothInstance {
    ClothTopology topology;
    ClothState state;
    ClothParams params;
};

class ClothWorld {
private:
    std::unordered_map<ClothID, ClothInstance> m_instances{};
    ClothID m_next_cloth_id{0};

public:
    ClothID create_cloth(ClothInstance instance) {
        const ClothID id = m_next_cloth_id++;
        m_instances.emplace(id, std::move(instance));
        return id;
    }

    bool remove_cloth(ClothID id) {
        return m_instances.erase(id) > 0u;
    }

    bool has_cloth(ClothID id) const {
        return m_instances.find(id) != m_instances.end();
    }

    ClothInstance& get_cloth(ClothID id) {
        const auto it = m_instances.find(id);
        if (it == m_instances.end()) {
            throw std::out_of_range("ClothID is invalid.");
        }
        return it->second;
    }

    const ClothInstance& get_cloth(ClothID id) const {
        const auto it = m_instances.find(id);
        if (it == m_instances.end()) {
            throw std::out_of_range("ClothID is invalid.");
        }
        return it->second;
    }

    std::size_t cloth_count() const {
        return m_instances.size();
    }

    void step(float /*delta_seconds*/) {
        // Phase 1 only wires the runtime skeleton; simulation is added later.
    }
};

}  // namespace rtr::system::physics
