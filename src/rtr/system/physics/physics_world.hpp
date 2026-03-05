#pragma once

#include <unordered_map>
#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/rigid_body.hpp"
namespace rtr::system::physics {

class PhysicsWorld {
private:
    std::unordered_map<RigidBodyID, RigidBody> m_rigid_bodies;
    
public:
    RigidBodyID create_rigid_body(RigidBody rigid_body) {
        RigidBodyID id = m_rigid_bodies.size();
        m_rigid_bodies[id] = std::move(rigid_body);
        return id;
    }   

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id); }

    RigidBody& get_rigid_body(RigidBodyID id) { return m_rigid_bodies.at(id); }
    const RigidBody& get_rigid_body(RigidBodyID id) const { return m_rigid_bodies.at(id); }

    void tick(float delta_seconds) {
        
    }

};


}

