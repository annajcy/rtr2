#pragma once

#include <cmath>
#include <optional>
#include <unordered_map>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision.hpp"
#include "rtr/system/physics/rigid_body.hpp"

namespace rtr::system::physics {

class PhysicsWorld {
private:
    struct BodyStepState {
        pbpt::math::Vec3 linear_acceleration{0.0f};
        pbpt::math::Vec3 angular_acceleration{0.0f};
        bool             has_angular_inertia{false};
    };

    std::unordered_map<RigidBodyID, RigidBody>  m_rigid_bodies{};
    std::unordered_map<ColliderID, Collider>    m_colliders{};
    RigidBodyID                                 m_next_rigid_body_id{0};
    ColliderID                                  m_next_collider_id{0};
    pbpt::math::Vec3                            m_gravity{0.0f, -9.81f, 0.0f};

    static bool is_valid_mass(pbpt::math::Float mass) {
        return std::isfinite(mass) && mass > 0.0f;
    }

    static bool is_finite_matrix(const pbpt::math::Mat3& matrix) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (!std::isfinite(matrix[row][col])) {
                    return false;
                }
            }
        }
        return true;
    }

    static pbpt::math::Quat integrate_orientation(const pbpt::math::Quat& orientation,
                                                  const pbpt::math::Vec3& angular_velocity,
                                                  float                   delta_seconds) {
        const pbpt::math::Quat omega_quat(0.0f, angular_velocity.x(), angular_velocity.y(), angular_velocity.z());
        const pbpt::math::Quat delta = (omega_quat * orientation) * (0.5f * delta_seconds);
        return pbpt::math::normalize(pbpt::math::Quat(orientation.w() + delta.w(), orientation.x() + delta.x(),
                                                      orientation.y() + delta.y(), orientation.z() + delta.z()));
    }

    static pbpt::math::Vec3 hadamard(const pbpt::math::Vec3& lhs, const pbpt::math::Vec3& rhs) {
        return pbpt::math::Vec3{lhs.x() * rhs.x(), lhs.y() * rhs.y(), lhs.z() * rhs.z()};
    }

    static pbpt::math::Vec3 abs_vec3(const pbpt::math::Vec3& value) {
        return pbpt::math::Vec3{std::abs(value.x()), std::abs(value.y()), std::abs(value.z())};
    }

    bool has_dynamic_body(const std::optional<RigidBodyID>& body_id) const {
        if (!body_id.has_value()) {
            return false;
        }
        const auto it = m_rigid_bodies.find(*body_id);
        if (it == m_rigid_bodies.end()) {
            return false;
        }
        const auto& body  = it->second;
        const auto& state = body.state();
        return body.type() == RigidBodyType::Dynamic && body.is_awake() && is_valid_mass(state.mass);
    }

    pbpt::math::Mat3 inverse_inertia_tensor_world(const RigidBody& body) const {
        const auto rotation_matrix = body.state().rotation.orientation.to_mat3();
        return rotation_matrix * body.inverse_inertia_tensor_ref() * pbpt::math::transpose(rotation_matrix);
    }

    void refresh_bound_collider_world_pose(Collider& collider) {
        if (!collider.rigid_body_id.has_value()) {
            return;
        }
        const auto body_it = m_rigid_bodies.find(*collider.rigid_body_id);
        if (body_it == m_rigid_bodies.end()) {
            collider.rigid_body_id.reset();
            return;
        }

        const auto& body   = body_it->second;
        const auto& state  = body.state();
        const auto  center = hadamard(collider.local_center, collider.world_scale);
        collider.world_position = state.translation.position + state.rotation.orientation * center;
        collider.world_rotation = pbpt::math::normalize(state.rotation.orientation * collider.local_rotation);
    }

    void refresh_dynamic_colliders() {
        for (auto& [collider_id, collider] : m_colliders) {
            (void)collider_id;
            if (!has_dynamic_body(collider.rigid_body_id)) {
                continue;
            }
            refresh_bound_collider_world_pose(collider);
        }
    }

    void integrate_forces_and_drift(float delta_seconds, std::unordered_map<RigidBodyID, BodyStepState>& step_states) {
        for (auto& [id, body] : m_rigid_bodies) {
            auto& state = body.state();
            if (body.type() != RigidBodyType::Dynamic || !body.is_awake() || !is_valid_mass(state.mass)) {
                continue;
            }

            const pbpt::math::Vec3 gravity_force =
                body.use_gravity() ? (m_gravity * state.mass) : pbpt::math::Vec3(0.0f);
            const pbpt::math::Vec3 total_force  = state.forces.accumulated_force + gravity_force;
            const pbpt::math::Vec3 acceleration = total_force / state.mass;

            if (!body.linear_half_step_initialized()) {
                body.initialize_half_step_linear_velocity(
                    state.translation.linear_velocity + acceleration * (0.5f * delta_seconds));
            }

            state.translation.position += body.half_step_linear_velocity() * delta_seconds;
            body.half_step_linear_velocity() += acceleration * delta_seconds;

            BodyStepState step_state{};
            step_state.linear_acceleration = acceleration;

            if (is_finite_matrix(body.inverse_inertia_tensor_ref())) {
                const auto inertia_world = inverse_inertia_tensor_world(body);
                const auto angular_acceleration = inertia_world * state.forces.accumulated_torque;
                if (!body.angular_half_step_initialized()) {
                    body.initialize_half_step_angular_velocity(
                        state.rotation.angular_velocity + angular_acceleration * (0.5f * delta_seconds));
                }

                state.rotation.orientation =
                    integrate_orientation(state.rotation.orientation, body.half_step_angular_velocity(), delta_seconds);
                body.half_step_angular_velocity() += angular_acceleration * delta_seconds;
                step_state.angular_acceleration = angular_acceleration;
                step_state.has_angular_inertia  = true;
            }

            step_states[id] = step_state;
        }
    }

    std::optional<Contact> generate_contact_for_pair(ColliderID a_id, const Collider& a, ColliderID b_id,
                                                     const Collider& b) const {
        if (a.rigid_body_id.has_value() && b.rigid_body_id.has_value() && a.rigid_body_id == b.rigid_body_id) {
            return std::nullopt;
        }

        const bool a_dynamic = has_dynamic_body(a.rigid_body_id);
        const bool b_dynamic = has_dynamic_body(b.rigid_body_id);
        if (!a_dynamic && !b_dynamic) {
            return std::nullopt;
        }

        if (std::holds_alternative<SphereShape>(a.shape) && std::holds_alternative<SphereShape>(b.shape)) {
            return collide_sphere_sphere(a_id, a, b_id, b);
        }

        if (std::holds_alternative<SphereShape>(a.shape) && std::holds_alternative<BoxShape>(b.shape)) {
            if (b_dynamic) {
                return std::nullopt;
            }
            return collide_sphere_box(a_id, a, b_id, b);
        }

        if (std::holds_alternative<BoxShape>(a.shape) && std::holds_alternative<SphereShape>(b.shape)) {
            if (a_dynamic) {
                return std::nullopt;
            }
            auto contact = collide_sphere_box(b_id, b, a_id, a);
            if (!contact.has_value()) {
                return std::nullopt;
            }
            return Contact{
                .body_a      = contact->body_b,
                .body_b      = contact->body_a,
                .collider_a  = a_id,
                .collider_b  = b_id,
                .point       = contact->point,
                .normal      = -contact->normal,
                .penetration = contact->penetration,
            };
        }

        return std::nullopt;
    }

    std::vector<Contact> generate_contacts() const {
        std::vector<Contact> contacts;
        for (auto it_a = m_colliders.begin(); it_a != m_colliders.end(); ++it_a) {
            auto it_b = it_a;
            ++it_b;
            for (; it_b != m_colliders.end(); ++it_b) {
                if (auto contact = generate_contact_for_pair(it_a->first, it_a->second, it_b->first, it_b->second);
                    contact.has_value()) {
                    contacts.push_back(*contact);
                }
            }
        }
        return contacts;
    }

    pbpt::math::Vec3 contact_linear_velocity(const RigidBody& body) const {
        return body.linear_half_step_initialized() ? body.half_step_linear_velocity()
                                                   : body.state().translation.linear_velocity;
    }

    pbpt::math::Vec3 contact_angular_velocity(const RigidBody& body) const {
        return body.angular_half_step_initialized() ? body.half_step_angular_velocity()
                                                    : body.state().rotation.angular_velocity;
    }

    void solve_contacts(const std::vector<Contact>& contacts) {
        constexpr pbpt::math::Float kPenetrationSlop    = 1e-3f;
        constexpr pbpt::math::Float kCorrectionPercent  = 0.8f;
        constexpr pbpt::math::Float kImpulseEpsilon     = 1e-6f;

        for (const auto& contact : contacts) {
            const auto normal_length = pbpt::math::length(contact.normal);
            if (normal_length <= kImpulseEpsilon) {
                continue;
            }
            const auto normal = contact.normal / normal_length;

            RigidBody* body_a = nullptr;
            RigidBody* body_b = nullptr;
            if (has_dynamic_body(contact.body_a)) {
                body_a = &m_rigid_bodies.at(*contact.body_a);
            }
            if (has_dynamic_body(contact.body_b)) {
                body_b = &m_rigid_bodies.at(*contact.body_b);
            }

            const pbpt::math::Float inv_mass_a =
                body_a != nullptr ? (1.0f / body_a->state().mass) : 0.0f;
            const pbpt::math::Float inv_mass_b =
                body_b != nullptr ? (1.0f / body_b->state().mass) : 0.0f;
            const pbpt::math::Float inv_mass_sum = inv_mass_a + inv_mass_b;
            if (inv_mass_sum <= kImpulseEpsilon) {
                continue;
            }

            const auto point_velocity_a = [&]() {
                if (body_a == nullptr) {
                    return pbpt::math::Vec3{0.0f};
                }
                const auto r = contact.point - body_a->state().translation.position;
                return contact_linear_velocity(*body_a) + pbpt::math::cross(contact_angular_velocity(*body_a), r);
            }();

            const auto point_velocity_b = [&]() {
                if (body_b == nullptr) {
                    return pbpt::math::Vec3{0.0f};
                }
                const auto r = contact.point - body_b->state().translation.position;
                return contact_linear_velocity(*body_b) + pbpt::math::cross(contact_angular_velocity(*body_b), r);
            }();

            const auto relative_velocity = point_velocity_b - point_velocity_a;
            const auto rel_normal_speed  = pbpt::math::dot(relative_velocity, normal);

            pbpt::math::Float angular_denom = 0.0f;
            if (body_a != nullptr && is_finite_matrix(body_a->inverse_inertia_tensor_ref())) {
                const auto r_a        = contact.point - body_a->state().translation.position;
                const auto r_a_cross_n = pbpt::math::cross(r_a, normal);
                angular_denom += pbpt::math::dot(
                    normal, pbpt::math::cross(inverse_inertia_tensor_world(*body_a) * r_a_cross_n, r_a));
            }
            if (body_b != nullptr && is_finite_matrix(body_b->inverse_inertia_tensor_ref())) {
                const auto r_b        = contact.point - body_b->state().translation.position;
                const auto r_b_cross_n = pbpt::math::cross(r_b, normal);
                angular_denom += pbpt::math::dot(
                    normal, pbpt::math::cross(inverse_inertia_tensor_world(*body_b) * r_b_cross_n, r_b));
            }

            const pbpt::math::Float denom = inv_mass_sum + angular_denom;
            if (denom > kImpulseEpsilon && rel_normal_speed < 0.0f) {
                const pbpt::math::Float impulse_magnitude = -rel_normal_speed / denom;
                const auto              impulse           = normal * impulse_magnitude;

                if (body_a != nullptr) {
                    body_a->half_step_linear_velocity() -= impulse * inv_mass_a;
                    if (is_finite_matrix(body_a->inverse_inertia_tensor_ref())) {
                        const auto r_a = contact.point - body_a->state().translation.position;
                        body_a->half_step_angular_velocity() -=
                            inverse_inertia_tensor_world(*body_a) * pbpt::math::cross(r_a, impulse);
                    }
                }
                if (body_b != nullptr) {
                    body_b->half_step_linear_velocity() += impulse * inv_mass_b;
                    if (is_finite_matrix(body_b->inverse_inertia_tensor_ref())) {
                        const auto r_b = contact.point - body_b->state().translation.position;
                        body_b->half_step_angular_velocity() +=
                            inverse_inertia_tensor_world(*body_b) * pbpt::math::cross(r_b, impulse);
                    }
                }
            }

            const auto correction_magnitude =
                kCorrectionPercent * std::max(contact.penetration - kPenetrationSlop, 0.0f) / inv_mass_sum;
            const auto correction = normal * correction_magnitude;
            if (body_a != nullptr) {
                body_a->state().translation.position -= correction * inv_mass_a;
            }
            if (body_b != nullptr) {
                body_b->state().translation.position += correction * inv_mass_b;
            }

            if (body_a != nullptr || body_b != nullptr) {
                refresh_dynamic_colliders();
            }
        }
    }

    void update_observable_velocities(float delta_seconds,
                                      const std::unordered_map<RigidBodyID, BodyStepState>& step_states) {
        const pbpt::math::Float half_dt = 0.5f * delta_seconds;
        for (const auto& [id, step_state] : step_states) {
            auto& body  = m_rigid_bodies.at(id);
            auto& state = body.state();
            state.translation.linear_velocity = body.half_step_linear_velocity() - step_state.linear_acceleration * half_dt;
            if (step_state.has_angular_inertia) {
                state.rotation.angular_velocity =
                    body.half_step_angular_velocity() - step_state.angular_acceleration * half_dt;
            }
        }
    }

public:
    RigidBodyID create_rigid_body(RigidBody rigid_body = RigidBody{}) {
        const RigidBodyID id = m_next_rigid_body_id++;
        m_rigid_bodies[id]   = std::move(rigid_body);
        return id;
    }

    ColliderID create_collider(Collider collider = Collider{}) {
        const ColliderID id = m_next_collider_id++;
        m_colliders[id]     = std::move(collider);
        return id;
    }

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id) > 0; }
    bool has_collider(ColliderID id) const { return m_colliders.count(id) > 0; }

    bool remove_rigid_body(RigidBodyID id) {
        for (auto& [collider_id, collider] : m_colliders) {
            (void)collider_id;
            if (collider.rigid_body_id == id) {
                collider.rigid_body_id.reset();
            }
        }
        return m_rigid_bodies.erase(id) > 0;
    }

    bool remove_collider(ColliderID id) { return m_colliders.erase(id) > 0; }

    RigidBody& get_rigid_body(RigidBodyID id) { return m_rigid_bodies.at(id); }
    const RigidBody& get_rigid_body(RigidBodyID id) const { return m_rigid_bodies.at(id); }

    Collider& get_collider(ColliderID id) { return m_colliders.at(id); }
    const Collider& get_collider(ColliderID id) const { return m_colliders.at(id); }

    const pbpt::math::Vec3& gravity() const { return m_gravity; }
    void set_gravity(const pbpt::math::Vec3& gravity) { m_gravity = gravity; }

    void tick(float delta_seconds) {
        std::unordered_map<RigidBodyID, BodyStepState> step_states;
        step_states.reserve(m_rigid_bodies.size());

        integrate_forces_and_drift(delta_seconds, step_states);
        refresh_dynamic_colliders();
        const auto contacts = generate_contacts();
        solve_contacts(contacts);
        refresh_dynamic_colliders();
        update_observable_velocities(delta_seconds, step_states);

        for (auto& [id, body] : m_rigid_bodies) {
            (void)id;
            body.clear_forces();
        }
    }
};

}  // namespace rtr::system::physics
