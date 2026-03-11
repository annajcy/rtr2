#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numeric>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/system/physics/collider.hpp"
#include "rtr/system/physics/collision.hpp"
#include "rtr/system/physics/rigid_body.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::physics {

class PhysicsWorld {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("system.physics.world");
    }

    static const char* rigid_body_type_name(RigidBodyType type) {
        switch (type) {
        case RigidBodyType::Static:
            return "Static";
        case RigidBodyType::Dynamic:
            return "Dynamic";
        case RigidBodyType::Kinematic:
            return "Kinematic";
        }
        return "Unknown";
    }

    static const char* collider_type_name(const ColliderShape& shape) {
        return std::visit(
            [](const auto& collider_shape) -> const char* {
                using Shape = std::decay_t<decltype(collider_shape)>;
                if constexpr (std::is_same_v<Shape, SphereShape>) {
                    return "Sphere";
                } else if constexpr (std::is_same_v<Shape, BoxShape>) {
                    return "Box";
                } else if constexpr (std::is_same_v<Shape, PlaneShape>) {
                    return "Plane";
                } else if constexpr (std::is_same_v<Shape, MeshShape>) {
                    return "Mesh";
                }
                return "Unknown";
            },
            shape
        );
    }

    static void log_contact_trace(const std::shared_ptr<spdlog::logger>& log,
                                  std::string_view phase,
                                  std::uint32_t iteration,
                                  const SolverContact& contact) {
        if (!log->should_log(spdlog::level::trace)) {
            return;
        }

        log->trace(
            "Solver contact state (phase={}, iteration={}, body_a={}, body_b={}, collider_a={}, collider_b={}, "
            "penetration={:.6f}, point=[{:.4f}, {:.4f}, {:.4f}], normal=[{:.4f}, {:.4f}, {:.4f}], "
            "tangent=[{:.4f}, {:.4f}, {:.4f}], normal_impulse_sum={:.6f}, tangent_impulse_sum={:.6f}).",
            phase,
            iteration,
            contact.body_a,
            contact.body_b,
            contact.collider_a,
            contact.collider_b,
            contact.penetration,
            contact.point.x(),
            contact.point.y(),
            contact.point.z(),
            contact.normal.x(),
            contact.normal.y(),
            contact.normal.z(),
            contact.tangent.x(),
            contact.tangent.y(),
            contact.tangent.z(),
            contact.normal_impulse_sum,
            contact.tangent_impulse_sum
        );
    }

    std::unordered_map<RigidBodyID, RigidBody>   m_rigid_bodies{};
    std::unordered_map<ColliderID, Collider>     m_colliders{};
    std::unordered_multimap<RigidBodyID, ColliderID> m_colliders_by_body{};
    RigidBodyID                                  m_next_rigid_body_id{0};
    ColliderID                                   m_next_collider_id{0};
    pbpt::math::Vec3                             m_gravity{0.0f, -9.81f, 0.0f};
    std::uint32_t                                m_velocity_iterations{8};
    std::uint32_t                                m_position_iterations{3};

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

    static pbpt::math::Vec3 fallback_tangent(const pbpt::math::Vec3& normal) {
        constexpr pbpt::math::Float kEpsilon = 1e-6f;
        const pbpt::math::Vec3 axis = std::abs(normal.x()) < 0.577f ? pbpt::math::Vec3{1.0f, 0.0f, 0.0f}
                                                                     : pbpt::math::Vec3{0.0f, 1.0f, 0.0f};
        auto tangent = pbpt::math::cross(normal, axis);
        const auto tangent_length_sq = pbpt::math::dot(tangent, tangent);
        if (tangent_length_sq <= kEpsilon * kEpsilon) {
            tangent = pbpt::math::cross(normal, pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
        }
        return pbpt::math::normalize(tangent);
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
        rb.state().translation.linear_velocity *= rb.linear_decay();
        rb.state().translation.position += rb.state().translation.linear_velocity * delta_seconds;

        const auto rotation_matrix = rb.state().rotation.orientation.to_mat3();
        const auto inv_inertia_tensor =
            rotation_matrix * rb.inverse_inertia_tensor_ref() * rotation_matrix.transposed();
        const auto angular_acc = inv_inertia_tensor * rb.state().forces.accumulated_torque;
        rb.state().rotation.angular_velocity += angular_acc * delta_seconds;
        rb.state().rotation.angular_velocity *= rb.angular_decay();

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

    std::vector<SolverContact> build_solver_contacts(const std::vector<Contact>& contacts) {
        constexpr pbpt::math::Float kTangentEpsilon = 1e-6f;
        std::vector<SolverContact> solver_contacts;
        solver_contacts.reserve(contacts.size());

        for (const auto& contact : contacts) {
            if (!contact.is_valid()) {
                continue;
            }

            const auto& body_a = get_rigid_body(contact.body_a);
            const auto& body_b = get_rigid_body(contact.body_b);
            const auto inv_mass_a = solver_inverse_mass(body_a);
            const auto inv_mass_b = solver_inverse_mass(body_b);
            const auto inv_mass_sum = inv_mass_a + inv_mass_b;
            if (inv_mass_sum <= 0.0f) {
                continue;
            }

            const auto normal = contact.result.normalized_normal();
            const auto point = contact.result.point;
            const auto r_a = contact_arm(body_a, point);
            const auto r_b = contact_arm(body_b, point);
            const auto inv_inertia_a = inverse_inertia_tensor_world(body_a);
            const auto inv_inertia_b = inverse_inertia_tensor_world(body_b);
            const auto effective_mass =
                effective_mass_matrix(inv_mass_a, inv_mass_b, inv_inertia_a, inv_inertia_b, r_a, r_b);
            const auto relative_velocity =
                contact_point_velocity(body_b, r_b) - contact_point_velocity(body_a, r_a);
            const auto tangent_velocity = relative_velocity - pbpt::math::dot(relative_velocity, normal) * normal;
            const auto tangent_speed_sq = pbpt::math::dot(tangent_velocity, tangent_velocity);
            const auto tangent = tangent_speed_sq > kTangentEpsilon * kTangentEpsilon
                                     ? tangent_velocity / std::sqrt(tangent_speed_sq)
                                     : fallback_tangent(normal);
            const auto normal_velocity = pbpt::math::dot(relative_velocity, normal);
            const auto restitution_bias =
                normal_velocity < 0.0f ? -combined_restitution(body_a, body_b) * normal_velocity : 0.0f;

            solver_contacts.push_back(SolverContact{
                .body_a = contact.body_a,
                .body_b = contact.body_b,
                .collider_a = contact.collider_a,
                .collider_b = contact.collider_b,
                .point = point,
                .normal = normal,
                .tangent = tangent,
                .penetration = contact.result.penetration,
                .r_a = r_a,
                .r_b = r_b,
                .inv_mass_a = inv_mass_a,
                .inv_mass_b = inv_mass_b,
                .inv_inertia_a = inv_inertia_a,
                .inv_inertia_b = inv_inertia_b,
                .effective_mass_normal = directional_effective_mass(effective_mass, normal),
                .effective_mass_tangent = directional_effective_mass(effective_mass, tangent),
                .restitution_bias = restitution_bias,
                .normal_impulse_sum = 0.0f,
                .tangent_impulse_sum = 0.0f,
            });
        }

        return solver_contacts;
    }

    void apply_position_correction(SolverContact& contact) {
        constexpr pbpt::math::Float kPositionCorrectionFactor = 0.8f;
        auto& body_a = get_rigid_body(contact.body_a);
        auto& body_b = get_rigid_body(contact.body_b);
        const auto inv_mass_sum = contact.inv_mass_a + contact.inv_mass_b;
        if (inv_mass_sum <= 0.0f || !contact.is_valid()) {
            return;
        }

        const auto corrected_penetration = kPositionCorrectionFactor * contact.penetration;
        const auto correction = (corrected_penetration / inv_mass_sum) * contact.normal;
        body_a.state().translation.position -= contact.inv_mass_a * correction;
        body_b.state().translation.position += contact.inv_mass_b * correction;
        contact.penetration = std::max(contact.penetration - corrected_penetration, 0.0f);
    }

    void apply_velocity_impulses(SolverContact& contact) {
        auto& body_a = get_rigid_body(contact.body_a);
        auto& body_b = get_rigid_body(contact.body_b);
        const auto inv_mass_sum = contact.inv_mass_a + contact.inv_mass_b;
        if (inv_mass_sum <= 0.0f || !contact.is_valid()) {
            return;
        }

        constexpr pbpt::math::Float kImpulseDenominatorEpsilon = 1e-6f;
        const auto relative_velocity =
            contact_point_velocity(body_b, contact.r_b) - contact_point_velocity(body_a, contact.r_a);

        if (contact.effective_mass_normal > kImpulseDenominatorEpsilon) {
            const auto normal_velocity = pbpt::math::dot(relative_velocity, contact.normal);
            const auto delta_normal_impulse =
                -(normal_velocity - contact.restitution_bias) / contact.effective_mass_normal;
            const auto previous_normal_impulse_sum = contact.normal_impulse_sum;
            const auto next_normal_impulse_sum = std::max(previous_normal_impulse_sum + delta_normal_impulse, 0.0f);
            const auto actual_delta_normal_impulse = next_normal_impulse_sum - previous_normal_impulse_sum;
            contact.normal_impulse_sum = next_normal_impulse_sum;

            const auto normal_impulse = actual_delta_normal_impulse * contact.normal;
            apply_impulse_at_contact(
                body_a, -normal_impulse, contact.r_a, contact.inv_mass_a, contact.inv_inertia_a);
            apply_impulse_at_contact(
                body_b, normal_impulse, contact.r_b, contact.inv_mass_b, contact.inv_inertia_b);
        }

        const auto relative_velocity_after_normal =
            contact_point_velocity(body_b, contact.r_b) - contact_point_velocity(body_a, contact.r_a);
        if (contact.effective_mass_tangent <= kImpulseDenominatorEpsilon) {
            return;
        }

        const auto delta_tangent_impulse =
            -pbpt::math::dot(relative_velocity_after_normal, contact.tangent) / contact.effective_mass_tangent;
        const auto previous_tangent_impulse_sum = contact.tangent_impulse_sum;
        const auto max_friction_impulse = combined_friction(body_a, body_b) * contact.normal_impulse_sum;
        const auto next_tangent_impulse_sum = pbpt::math::clamp(
            previous_tangent_impulse_sum + delta_tangent_impulse, -max_friction_impulse, max_friction_impulse);
        const auto actual_delta_tangent_impulse = next_tangent_impulse_sum - previous_tangent_impulse_sum;
        contact.tangent_impulse_sum = next_tangent_impulse_sum;

        const auto tangent_impulse = actual_delta_tangent_impulse * contact.tangent;
        apply_impulse_at_contact(
            body_a, -tangent_impulse, contact.r_a, contact.inv_mass_a, contact.inv_inertia_a);
        apply_impulse_at_contact(
            body_b, tangent_impulse, contact.r_b, contact.inv_mass_b, contact.inv_inertia_b);
    }

public:
    static constexpr std::uint32_t kDefaultVelocityIterations = 8;
    static constexpr std::uint32_t kDefaultPositionIterations = 3;

    RigidBodyID create_rigid_body(RigidBody rigid_body = RigidBody{}) {
        const RigidBodyID id = m_next_rigid_body_id++;
        m_rigid_bodies[id]   = std::move(rigid_body);
        logger()->debug("Rigid body created (rigid_body_id={}, type={}, awake={}, mass={}, body_count={}).",
                        id,
                        rigid_body_type_name(m_rigid_bodies.at(id).type()),
                        m_rigid_bodies.at(id).is_awake(),
                        m_rigid_bodies.at(id).state().mass,
                        m_rigid_bodies.size());
        return id;
    }

    ColliderID create_collider(RigidBodyID owner_body_id, Collider collider = Collider{}) {
        if (!has_rigid_body(owner_body_id)) {
            logger()->error("create_collider failed: owner rigid body {} does not exist.", owner_body_id);
            throw std::out_of_range("Collider owner body does not exist.");
        }

        const ColliderID id = m_next_collider_id++;
        collider.rigid_body_id = owner_body_id;
        m_colliders[id]        = std::move(collider);
        m_colliders_by_body.emplace(owner_body_id, id);
        logger()->debug("Collider created (collider_id={}, owner_rigid_body_id={}, type={}, collider_count={}).",
                        id,
                        owner_body_id,
                        collider_type_name(m_colliders.at(id).shape),
                        m_colliders.size());
        return id;
    }

    bool has_rigid_body(RigidBodyID id) const { return m_rigid_bodies.count(id) > 0; }
    bool has_collider(ColliderID id) const { return m_colliders.count(id) > 0; }

    bool remove_rigid_body(RigidBodyID id) {
        if (!has_rigid_body(id)) {
            logger()->warn("remove_rigid_body ignored: rigid body {} does not exist.", id);
            return false;
        }

        const auto collider_ids = colliders_for_body(id);
        for (const auto collider_id : collider_ids) {
            remove_collider_internal(collider_id);
        }
        const auto removed = m_rigid_bodies.erase(id) > 0;
        if (removed) {
            logger()->debug("Rigid body removed (rigid_body_id={}, removed_colliders={}, body_count={}).",
                            id,
                            collider_ids.size(),
                            m_rigid_bodies.size());
        }
        return removed;
    }

    bool remove_collider(ColliderID id) {
        if (!has_collider(id)) {
            logger()->debug("remove_collider ignored: collider {} does not exist.", id);
            return false;
        }
        const auto owner_body_id = get_collider(id).rigid_body_id;
        remove_collider_internal(id);
        logger()->debug("Collider removed (collider_id={}, owner_rigid_body_id={}, collider_count={}).",
                        id,
                        owner_body_id,
                        m_colliders.size());
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
    void set_gravity(const pbpt::math::Vec3& gravity) {
        m_gravity = gravity;
        logger()->info("Gravity updated to [{:.3f}, {:.3f}, {:.3f}].", gravity.x(), gravity.y(), gravity.z());
    }
    std::uint32_t velocity_iterations() const { return m_velocity_iterations; }
    std::uint32_t position_iterations() const { return m_position_iterations; }

    void set_velocity_iterations(std::uint32_t iterations) {
        if (iterations == 0) {
            logger()->error("set_velocity_iterations failed: iterations must be greater than zero.");
            throw std::invalid_argument("PhysicsWorld velocity_iterations must be greater than zero.");
        }
        m_velocity_iterations = iterations;
        logger()->info("Velocity iterations updated to {}.", m_velocity_iterations);
    }

    void set_position_iterations(std::uint32_t iterations) {
        if (iterations == 0) {
            logger()->error("set_position_iterations failed: iterations must be greater than zero.");
            throw std::invalid_argument("PhysicsWorld position_iterations must be greater than zero.");
        }
        m_position_iterations = iterations;
        logger()->info("Position iterations updated to {}.", m_position_iterations);
    }

    void tick(float delta_seconds) {
        auto log = logger();
        for (auto& [id, rb] : m_rigid_bodies) {
            integrate_body(rb, delta_seconds);
        }

        const auto contacts = collect_contacts();
        auto solver_contacts = build_solver_contacts(contacts);
        const auto contact_snapshot_count = solver_contacts.size();
        const auto max_penetration_snapshot = std::accumulate(
            solver_contacts.begin(),
            solver_contacts.end(),
            pbpt::math::Float{0.0f},
            [](const pbpt::math::Float current_max, const SolverContact& contact) {
                return std::max(current_max, contact.penetration);
            });

        std::uint32_t velocity_iterations_executed = 0;
        std::uint32_t position_iterations_executed = 0;
        if (!solver_contacts.empty()) {
            for (std::uint32_t iteration = 0; iteration < m_velocity_iterations; ++iteration) {
                ++velocity_iterations_executed;
                for (auto& contact : solver_contacts) {
                    apply_velocity_impulses(contact);
                    log_contact_trace(log, "velocity", iteration, contact);
                }
            }

            for (std::uint32_t iteration = 0; iteration < m_position_iterations; ++iteration) {
                ++position_iterations_executed;
                for (auto& contact : solver_contacts) {
                    apply_position_correction(contact);
                }
            }
        }

        for (auto& [id, rb] : m_rigid_bodies) {
            (void)id;
            rb.clear_forces();
        }

        if (!solver_contacts.empty() && log->should_log(spdlog::level::debug)) {
            pbpt::math::Float max_normal_impulse_sum = 0.0f;
            pbpt::math::Float max_abs_tangent_impulse_sum = 0.0f;
            for (const auto& contact : solver_contacts) {
                max_normal_impulse_sum = std::max(max_normal_impulse_sum, contact.normal_impulse_sum);
                max_abs_tangent_impulse_sum = std::max(max_abs_tangent_impulse_sum, std::abs(contact.tangent_impulse_sum));
            }

            log->debug("Collision solve summary (delta_seconds={:.6f}, contact_snapshot_count={}, "
                       "velocity_iterations={}, position_iterations={}, "
                       "max_penetration_snapshot={:.6f}, max_normal_impulse_sum={:.6f}, "
                       "max_abs_tangent_impulse_sum={:.6f}).",
                       delta_seconds,
                       contact_snapshot_count,
                       m_velocity_iterations,
                       m_position_iterations,
                       max_penetration_snapshot,
                       max_normal_impulse_sum,
                       max_abs_tangent_impulse_sum);
        }

        log->trace("Physics tick completed (delta_seconds={:.6f}, contact_snapshot_count={}, "
                   "velocity_iterations_executed={}, position_iterations_executed={}, "
                   "rigid_body_count={}, collider_count={}).",
                   delta_seconds,
                   contact_snapshot_count,
                   velocity_iterations_executed,
                   position_iterations_executed,
                   m_rigid_bodies.size(),
                   m_colliders.size());
    }
};

}  // namespace rtr::system::physics
