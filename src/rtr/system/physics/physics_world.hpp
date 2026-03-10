#pragma once

#include <cmath>
#include <optional>
#include <stdexcept>
#include <type_traits>
#include <unordered_map>
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

    static pbpt::math::Float solver_inverse_mass(const RigidBody& body) {
        if (body.type() != RigidBodyType::Dynamic || !body.is_awake() || !body.state().is_mass_valid()) {
            return 0.0f;
        }
        return body.state().inverse_mass();
    }

    static pbpt::math::Float combined_restitution(const RigidBody& body_a, const RigidBody& body_b) {
        return std::max(body_a.restitution(), body_b.restitution());
    }

    static pbpt::math::Float combined_friction(const RigidBody& body_a, const RigidBody& body_b) {
        return std::sqrt(body_a.friction() * body_b.friction());
    }

    static pbpt::math::Mat3 inverse_inertia_tensor_world(const RigidBody& body) {
        if (solver_inverse_mass(body) <= 0.0f) {
            return pbpt::math::Mat3::zeros();
        }

        const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
        return rotation_matrix * body.inverse_inertia_tensor_ref() * rotation_matrix.transposed();
    }

    static pbpt::math::Vec3 contact_arm(const RigidBody& body, const pbpt::math::Vec3& contact_point) {
        return contact_point - body.state().translation.position;
    }

    static pbpt::math::Vec3 contact_point_velocity(const RigidBody& body, const pbpt::math::Vec3& r) {
        return body.state().translation.linear_velocity + pbpt::math::cross(body.state().rotation.angular_velocity, r);
    }

    static pbpt::math::Mat3 effective_mass_matrix(const pbpt::math::Float inv_mass_a,
                                                  const pbpt::math::Float inv_mass_b,
                                                  const pbpt::math::Mat3& inv_inertia_a,
                                                  const pbpt::math::Mat3& inv_inertia_b,
                                                  const pbpt::math::Vec3& r_a,
                                                  const pbpt::math::Vec3& r_b) {
        const auto identity = pbpt::math::Mat3::identity();
        const auto r_a_star = pbpt::math::cross_matrix(r_a);
        const auto r_b_star = pbpt::math::cross_matrix(r_b);
        return (inv_mass_a + inv_mass_b) * identity - r_a_star * inv_inertia_a * r_a_star -
               r_b_star * inv_inertia_b * r_b_star;
    }

    static pbpt::math::Float directional_effective_mass(const pbpt::math::Mat3& effective_mass,
                                                        const pbpt::math::Vec3& direction) {
        return pbpt::math::dot(direction, effective_mass * direction);
    }

    static void apply_impulse_at_contact(RigidBody& body,
                                         const pbpt::math::Vec3& impulse,
                                         const pbpt::math::Vec3& r,
                                         const pbpt::math::Float inv_mass,
                                         const pbpt::math::Mat3& inv_inertia_tensor) {
        if (inv_mass <= 0.0f) {
            return;
        }

        body.state().translation.linear_velocity += inv_mass * impulse;
        body.state().rotation.angular_velocity += inv_inertia_tensor * pbpt::math::cross(r, impulse);
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

    std::optional<Contact> generate_contact_from_pair(ColliderID collider_id_a, ColliderID collider_id_b) {
        auto collider_a = get_world_collider(collider_id_a);
        auto collider_b = get_world_collider(collider_id_b); 

        ContactResult result;
        std::visit([&](auto&& collider_shape_a, auto&& collider_shape_b){
            result = ContactPairTrait<
                std::decay_t<decltype(collider_shape_a)>,
                std::decay_t<decltype(collider_shape_b)>>::generate(collider_shape_a, collider_shape_b);
        }, collider_a, collider_b);

        if (result.is_valid()) {
            const auto& collider_record_a = get_collider(collider_id_a);
            const auto& collider_record_b = get_collider(collider_id_b);
            return Contact{
                .body_a = collider_record_a.rigid_body_id,
                .body_b = collider_record_b.rigid_body_id,
                .collider_a = collider_id_a,
                .collider_b = collider_id_b,
                .result = result
            };
        } else {
            return std::nullopt;
        }
    }

    void integrate_body(RigidBody& rb, float delta_seconds) {
        if (rb.type() != RigidBodyType::Dynamic || !rb.is_awake()) {
            return;
        }

        if (rb.use_gravity()) {
            rb.state().forces.accumulated_force += gravity() * rb.state().mass;
        }

        const auto acc = rb.state().inverse_mass() * rb.state().forces.accumulated_force;
        rb.state().translation.linear_velocity += acc * delta_seconds;
        rb.state().translation.position += rb.state().translation.linear_velocity * delta_seconds;

        const auto rotation_matrix = rb.state().rotation.orientation.to_mat3();
        const auto inv_inertia_tensor =
            rotation_matrix * rb.inverse_inertia_tensor_ref() * rotation_matrix.transposed();
        const auto angular_acc = inv_inertia_tensor * rb.state().forces.accumulated_torque;
        rb.state().rotation.angular_velocity += angular_acc * delta_seconds;

        const auto angular_velocity_quat = pbpt::math::Quat(0.0f,
                                                            rb.state().rotation.angular_velocity.x(),
                                                            rb.state().rotation.angular_velocity.y(),
                                                            rb.state().rotation.angular_velocity.z());
        rb.state().rotation.orientation += 0.5f * delta_seconds * angular_velocity_quat * rb.state().rotation.orientation;
        rb.state().rotation.orientation = pbpt::math::normalize(rb.state().rotation.orientation);
    }

    std::vector<Contact> collect_contacts() {
        std::vector<Contact> contacts;
        std::vector<ColliderID> collider_ids;
        collider_ids.reserve(m_colliders.size());
        for (const auto& [id, collider] : m_colliders) {
            (void)collider;
            collider_ids.push_back(id);
        }

        for (std::size_t i = 0; i < collider_ids.size(); ++i) {
            for (std::size_t j = i + 1; j < collider_ids.size(); ++j) {
                const auto collider_a_id = collider_ids[i];
                const auto collider_b_id = collider_ids[j];
                const auto& collider_a = get_collider(collider_a_id);
                const auto& collider_b = get_collider(collider_b_id);
                if (collider_a.rigid_body_id == collider_b.rigid_body_id) {
                    continue;
                }

                if (!has_dynamic_body(collider_a.rigid_body_id) && !has_dynamic_body(collider_b.rigid_body_id)) {
                    continue;
                }

                if (auto contact = generate_contact_from_pair(collider_a_id, collider_b_id); contact.has_value()) {
                    contacts.push_back(*contact);
                }
            }
        }

        return contacts;
    }

    void apply_position_correction(const Contact& contact) {
        auto& body_a = get_rigid_body(contact.body_a);
        auto& body_b = get_rigid_body(contact.body_b);
        const auto inv_mass_a = solver_inverse_mass(body_a);
        const auto inv_mass_b = solver_inverse_mass(body_b);
        const auto inv_mass_sum = inv_mass_a + inv_mass_b;
        if (inv_mass_sum <= 0.0f || !contact.is_valid()) {
            return;
        }

        const auto normal = contact.result.normalized_normal();
        const auto correction = (contact.result.penetration / inv_mass_sum) * normal;
        body_a.state().translation.position -= inv_mass_a * correction;
        body_b.state().translation.position += inv_mass_b * correction;
    }

    void apply_velocity_impulses(const Contact& contact) {
        auto& body_a = get_rigid_body(contact.body_a);
        auto& body_b = get_rigid_body(contact.body_b);
        const auto inv_mass_a = solver_inverse_mass(body_a);
        const auto inv_mass_b = solver_inverse_mass(body_b);
        const auto inv_mass_sum = inv_mass_a + inv_mass_b;
        if (inv_mass_sum <= 0.0f || !contact.is_valid()) {
            return;
        }

        constexpr pbpt::math::Float kImpulseDenominatorEpsilon = 1e-6f;
        constexpr pbpt::math::Float kTangentEpsilon = 1e-6f;
        const auto normal = contact.result.normalized_normal();
        const auto contact_point  = contact.result.point;
        const auto r_a            = contact_arm(body_a, contact_point);
        const auto r_b            = contact_arm(body_b, contact_point);
        const auto inv_inertia_a  = inverse_inertia_tensor_world(body_a);
        const auto inv_inertia_b  = inverse_inertia_tensor_world(body_b);
        const auto effective_mass = effective_mass_matrix(inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b, r_a, r_b);

        const auto relative_velocity =
            contact_point_velocity(body_b, r_b) - contact_point_velocity(body_a, r_a);
        const auto normal_velocity = pbpt::math::dot(relative_velocity, normal);
        if (normal_velocity >= 0.0f) {
            return;
        }

        const auto normal_effective_mass = directional_effective_mass(effective_mass, normal);
        if (normal_effective_mass <= kImpulseDenominatorEpsilon) {
            return;
        }

        const auto normal_impulse_magnitude =
            -((1.0f + combined_restitution(body_a, body_b)) * normal_velocity) / normal_effective_mass;
        const auto normal_impulse = normal_impulse_magnitude * normal;
        apply_impulse_at_contact(body_a, -normal_impulse, r_a, inv_mass_a, inv_inertia_a);
        apply_impulse_at_contact(body_b, normal_impulse, r_b, inv_mass_b, inv_inertia_b);

        const auto relative_velocity_after_normal =
            contact_point_velocity(body_b, r_b) - contact_point_velocity(body_a, r_a);
        const auto tangent_velocity = relative_velocity_after_normal -
                                      pbpt::math::dot(relative_velocity_after_normal, normal) * normal;
        const auto tangent_speed_sq = pbpt::math::dot(tangent_velocity, tangent_velocity);
        if (tangent_speed_sq <= kTangentEpsilon * kTangentEpsilon) {
            return;
        }

        const auto tangent = tangent_velocity / std::sqrt(tangent_speed_sq);
        const auto tangent_effective_mass = directional_effective_mass(effective_mass, tangent);
        if (tangent_effective_mass <= kImpulseDenominatorEpsilon) {
            return;
        }

        auto tangent_impulse_magnitude =
            -pbpt::math::dot(relative_velocity_after_normal, tangent) / tangent_effective_mass;
        const auto max_friction_impulse =
            combined_friction(body_a, body_b) * std::abs(normal_impulse_magnitude);
        tangent_impulse_magnitude = pbpt::math::clamp(
            tangent_impulse_magnitude, -max_friction_impulse, max_friction_impulse);

        const auto tangent_impulse = tangent_impulse_magnitude * tangent;
        apply_impulse_at_contact(body_a, -tangent_impulse, r_a, inv_mass_a, inv_inertia_a);
        apply_impulse_at_contact(body_b, tangent_impulse, r_b, inv_mass_b, inv_inertia_b);
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
        for (auto& [id, rb] : m_rigid_bodies) {
            integrate_body(rb, delta_seconds);
        }

        const auto contacts = collect_contacts();
        for (const auto& contact : contacts) {
            apply_position_correction(contact);
        }
        for (const auto& contact : contacts) {
            apply_velocity_impulses(contact);
        }

        for (auto& [id, rb] : m_rigid_bodies) {
            (void)id;
            rb.clear_forces();
        }
    }
};

}  // namespace rtr::system::physics
