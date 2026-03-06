#pragma once

#include <unordered_map>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/rigid_body.hpp"

namespace rtr::system::physics {

class PhysicsWorld {
private:
    std::unordered_map<RigidBodyID, RigidBody> m_rigid_bodies;
    RigidBodyID m_next_rigid_body_id{0};

public:
    RigidBodyID create_rigid_body(RigidBody rigid_body = RigidBody{}) {
        const RigidBodyID id = m_next_rigid_body_id++;
        m_rigid_bodies[id] = std::move(rigid_body);
        return id;
    }

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id); }
    bool remove_rigid_body(RigidBodyID id) { return m_rigid_bodies.erase(id) > 0; }

    RigidBody& get_rigid_body(RigidBodyID id) { return m_rigid_bodies.at(id); }
    const RigidBody& get_rigid_body(RigidBodyID id) const { return m_rigid_bodies.at(id); }

    void tick(float delta_seconds) {
        for (auto& [id, body] : m_rigid_bodies) {
            (void)id;
            auto& state = body.state();
            state.translation_state.position += state.translation_state.linear_velocity * delta_seconds;
        }
    }
};

}  // namespace rtr::system::physics
