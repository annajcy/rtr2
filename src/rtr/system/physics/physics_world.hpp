#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <variant>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision.hpp"
#include "rtr/system/physics/rigid_body.hpp"

namespace rtr::system::physics {

class PhysicsWorld {
private:
    std::unordered_map<RigidBodyID, RigidBody>   m_rigid_bodies{};
    std::unordered_map<ColliderID, Collider>     m_colliders{};
    std::unordered_multimap<RigidBodyID, ColliderID> m_colliders_by_body{};
    RigidBodyID                                  m_next_rigid_body_id{0};
    ColliderID                                   m_next_collider_id{0};
    pbpt::math::Vec3                             m_gravity{0.0f, -9.81f, 0.0f};

    bool has_dynamic_body(RigidBodyID body_id) const {
        if (body_id == kInvalidRigidBodyId) {
            return false;
        }
        const auto it = m_rigid_bodies.find(body_id);
        if (it == m_rigid_bodies.end()) {
            return false;
        }
        const auto& body  = it->second;
        const auto& state = body.state();
        return body.type() == RigidBodyType::Dynamic && body.is_awake() && state.is_mass_valid();
    }

    bool remove_body_index_entry(RigidBodyID body_id, ColliderID collider_id) {
        const auto [begin, end] = m_colliders_by_body.equal_range(body_id);
        for (auto it = begin; it != end; ++it) {
            if (it->second == collider_id) {
                m_colliders_by_body.erase(it);
                return true;
            }
        }
        return false;
    }

    bool remove_collider_internal(ColliderID id) {
        const auto it = m_colliders.find(id);
        if (it == m_colliders.end()) {
            return false;
        }
        auto body_index_removed = remove_body_index_entry(it->second.rigid_body_id, id);
        m_colliders.erase(it);
        return body_index_removed;
    }


    std::optional<Contact> generate_contact_from_pair(ColliderID a_id, ColliderID b_id) {
        auto collider_a = get_world_collider(a_id);
        auto collider_b = get_world_collider(b_id); 

        ContactResult result;
        std::visit([&](auto&& collider_shape_a, auto&& collider_shape_b){
            result = ContactPairTrait<
                std::decay_t<decltype(collider_shape_a)>,
                std::decay_t<decltype(collider_shape_b)>>::generate(collider_shape_a, collider_shape_b);
        }, collider_a, collider_b);

        if (result.is_valid()) {
            return Contact{
                .body_a = a_id,
                .body_b = b_id,
                .collider_a = a_id,
                .collider_b = b_id,
                .result = result
            };
        } else {
            return std::nullopt;
        }
    }

public:
    RigidBodyID create_rigid_body(RigidBody rigid_body = RigidBody{}) {
        const RigidBodyID id = m_next_rigid_body_id++;
        m_rigid_bodies[id]   = std::move(rigid_body);
        return id;
    }

    ColliderID create_collider(RigidBodyID owner_body_id, Collider collider = Collider{}) {
        if (!has_rigid_body(owner_body_id)) {
            throw std::out_of_range("Collider owner body does not exist.");
        }

        const ColliderID id = m_next_collider_id++;
        collider.rigid_body_id = owner_body_id;
        m_colliders[id]        = std::move(collider);
        m_colliders_by_body.emplace(owner_body_id, id);
        return id;
    }

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id) > 0; }
    bool has_collider(ColliderID id) const { return m_colliders.count(id) > 0; }

    bool remove_rigid_body(RigidBodyID id) {
        if (!has_rigid_body(id)) {
            return false;
        }

        const auto collider_ids = colliders_for_body(id);
        for (const auto collider_id : collider_ids) {
            remove_collider_internal(collider_id);
        }
        return m_rigid_bodies.erase(id) > 0;
    }

    bool remove_collider(ColliderID id) {
        if (!has_collider(id)) {
            return false;
        }
        remove_collider_internal(id);
        return true;
    }

    RigidBody& get_rigid_body(RigidBodyID id) { return m_rigid_bodies.at(id); }
    const RigidBody& get_rigid_body(RigidBodyID id) const { return m_rigid_bodies.at(id); }

    Collider& get_collider(ColliderID id) { return m_colliders.at(id); }
    const Collider& get_collider(ColliderID id) const { return m_colliders.at(id); }

    WorldCollider get_world_collider(ColliderID id) const { 
        const auto& collider = get_collider(id);
        WorldCollider world_collider = std::visit([&](auto&& collider_shape) ->WorldCollider {
            return to_world_collider(collider_shape, collider.local_transform, get_rigid_body(collider.rigid_body_id).state().to_transform());
        }, collider.shape);
        return world_collider;
    }

    std::vector<ColliderID> colliders_for_body(RigidBodyID id) const {
        std::vector<ColliderID> result;
        const auto [begin, end] = m_colliders_by_body.equal_range(id);
        for (auto it = begin; it != end; ++it) {
            result.push_back(it->second);
        }
        return result;
    }

    const pbpt::math::Vec3& gravity() const { return m_gravity; }
    void set_gravity(const pbpt::math::Vec3& gravity) { m_gravity = gravity; }

    void tick(float delta_seconds) {
        // update forces of dynamic bodies
        for (auto &[id, rb]: m_rigid_bodies) {
            if (rb.type() == RigidBodyType::Dynamic && rb.is_awake()) {
                if (rb.use_gravity()) {
                    rb.state().forces.accumulated_force += gravity() * rb.state().mass;
                }
            }

            // update acceleration and velocity
            auto acc = rb.state().inverse_mass() * rb.state().forces.accumulated_force;
            rb.state().translation.linear_velocity += acc * delta_seconds;
            rb.state().translation.position += rb.state().translation.linear_velocity * delta_seconds;

            //update angular dynamics
            auto inv_inertia_tensor = rb.state().rotation.orientation.to_mat3() * rb.inverse_inertia_tensor_ref() * rb.state().rotation.orientation.to_mat3().transpose();
            auto angular_acc = inv_inertia_tensor * rb.state().forces.accumulated_torque;
            rb.state().rotation.angular_velocity += angular_acc * delta_seconds;
            auto angular_velocity_quat = pbpt::math::Quat(0.0f, rb.state().rotation.angular_velocity.x(), rb.state().rotation.angular_velocity.y(), rb.state().rotation.angular_velocity.z());
            rb.state().rotation.orientation += 0.5f * delta_seconds * angular_velocity_quat * rb.state().rotation.orientation;
            rb.state().rotation.orientation = pbpt::math::normalize(rb.state().rotation.orientation);

            // clear forces for next tick
            rb.clear_forces();

            // handle collision
            
        }
    }
};

}  // namespace rtr::system::physics
