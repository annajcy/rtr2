#pragma once

#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/physics/rigid_body/rigid_body.hpp"

namespace rtr::framework::component {

class RigidBody final : public Component {
public:
    enum class PendingRigidBodyCommandType {
        AddForce,
        AddTorque,
        AddForceAtPoint,
        ClearForces,
        ResetDynamics,
        ResetTranslationDynamics,
        ResetRotationalDynamics,
    };

    struct PendingRigidBodyCommand {
        PendingRigidBodyCommandType type{};
        pbpt::math::Vec3 value{0.0f};
        pbpt::math::Vec3 point{0.0f};
    };

private:
    system::physics::rb::RigidBody m_source_body{};
    bool m_source_dirty{false};
    bool m_transform_dirty{false};
    bool m_lifecycle_dirty{false};
    bool m_should_exist_in_runtime{false};
    std::vector<PendingRigidBodyCommand> m_pending_commands{};

    static pbpt::math::Float sanitize_mass(pbpt::math::Float mass) {
        if (!std::isfinite(mass) || mass <= 0.0f) {
            throw std::invalid_argument("RigidBody mass must be finite and positive.");
        }
        return mass;
    }

    static pbpt::math::Mat3 sanitize_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        for (int row = 0; row < 3; ++row) {
            for (int col = 0; col < 3; ++col) {
                if (!std::isfinite(inverse_inertia_tensor_ref[row][col])) {
                    throw std::invalid_argument("RigidBody inverse inertia tensor must be finite.");
                }
            }
        }
        return inverse_inertia_tensor_ref;
    }

    static pbpt::math::Float sanitize_restitution(pbpt::math::Float restitution) {
        if (!std::isfinite(restitution) || restitution < 0.0f || restitution > 1.0f) {
            throw std::invalid_argument("RigidBody restitution must be finite and within [0, 1].");
        }
        return restitution;
    }

    static pbpt::math::Float sanitize_friction(pbpt::math::Float friction) {
        if (!std::isfinite(friction) || friction < 0.0f) {
            throw std::invalid_argument("RigidBody friction must be finite and non-negative.");
        }
        return friction;
    }

    static pbpt::math::Float sanitize_decay(pbpt::math::Float decay, const char* label) {
        if (!std::isfinite(decay) || decay < 0.0f || decay > 1.0f) {
            throw std::invalid_argument(std::string(label) + " must be finite and within [0, 1].");
        }
        return decay;
    }

    static system::physics::rb::RigidBody sanitize_source_body(system::physics::rb::RigidBody source_body) {
        source_body.set_awake(true);
        source_body.state().mass = sanitize_mass(source_body.state().mass);
        source_body.set_restitution(sanitize_restitution(source_body.restitution()));
        source_body.set_friction(sanitize_friction(source_body.friction()));
        source_body.set_linear_decay(sanitize_decay(source_body.linear_decay(), "RigidBody linear_decay"));
        source_body.set_angular_decay(sanitize_decay(source_body.angular_decay(), "RigidBody angular_decay"));
        source_body.set_inverse_inertia_tensor_ref(
            sanitize_inverse_inertia_tensor_ref(source_body.inverse_inertia_tensor_ref())
        );
        return source_body;
    }

    static system::physics::rb::RigidBody make_default_source_body() {
        system::physics::rb::RigidBody source_body{};
        source_body.state().mass = 1.0f;
        source_body.set_type(system::physics::rb::RigidBodyType::Dynamic);
        source_body.set_use_gravity(true);
        source_body.set_restitution(0.0f);
        source_body.set_friction(0.0f);
        source_body.set_linear_decay(1.0f);
        source_body.set_angular_decay(1.0f);
        source_body.set_inverse_inertia_tensor_ref(pbpt::math::Mat3::zeros());
        return source_body;
    }

    void enqueue_command(PendingRigidBodyCommand command) { m_pending_commands.push_back(std::move(command)); }

public:
    explicit RigidBody(core::GameObject& owner)
        : Component(owner),
          m_source_body(sanitize_source_body(make_default_source_body())) {}

    explicit RigidBody(core::GameObject& owner, system::physics::rb::RigidBody source_body)
        : Component(owner),
          m_source_body(sanitize_source_body(std::move(source_body))) {}

    void on_awake() override {}

    void on_enable() override {
        m_should_exist_in_runtime = true;
        m_lifecycle_dirty = true;
    }

    void on_disable() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }

    void on_destroy() override {
        m_should_exist_in_runtime = false;
        m_lifecycle_dirty = true;
    }

    const system::physics::rb::RigidBody& source_body() const { return m_source_body; }

    system::physics::rb::RigidBody make_runtime_body_snapshot() const {
        auto runtime_body = m_source_body;
        runtime_body.set_awake(true);
        runtime_body.state().translation.position = owner().node().world_position();
        runtime_body.state().rotation.orientation = owner().node().world_rotation();
        runtime_body.state().scale = owner().node().world_scale();
        runtime_body.clear_forces();
        return runtime_body;
    }

    bool source_dirty() const { return m_source_dirty; }
    bool transform_dirty() const { return m_transform_dirty; }
    bool lifecycle_dirty() const { return m_lifecycle_dirty; }
    bool should_exist_in_runtime() const { return m_should_exist_in_runtime; }
    bool has_pending_commands() const { return !m_pending_commands.empty(); }
    const std::vector<PendingRigidBodyCommand>& pending_commands() const { return m_pending_commands; }

    void clear_source_dirty() { m_source_dirty = false; }
    void clear_transform_dirty() { m_transform_dirty = false; }
    void clear_lifecycle_dirty() { m_lifecycle_dirty = false; }
    void clear_pending_commands() { m_pending_commands.clear(); }

    void sync_runtime_state_from(const system::physics::rb::RigidBody& runtime_body) {
        m_source_body.state().translation.linear_velocity = runtime_body.state().translation.linear_velocity;
        m_source_body.state().rotation.angular_velocity = runtime_body.state().rotation.angular_velocity;
    }

    system::physics::rb::RigidBodyType type() const { return m_source_body.type(); }
    void set_type(system::physics::rb::RigidBodyType type) {
        m_source_body.set_type(type);
        m_source_dirty = true;
    }

    pbpt::math::Vec3 position() const { return owner().node().world_position(); }
    void set_position(const pbpt::math::Vec3& position) {
        owner().node().set_world_position(position);
        m_transform_dirty = true;
    }

    pbpt::math::Vec3 linear_velocity() const { return m_source_body.state().translation.linear_velocity; }
    void set_linear_velocity(const pbpt::math::Vec3& linear_velocity) {
        m_source_body.state().translation.linear_velocity = linear_velocity;
        m_source_dirty = true;
    }

    pbpt::math::Quat orientation() const { return owner().node().world_rotation(); }
    void set_orientation(const pbpt::math::Quat& orientation) {
        owner().node().set_world_rotation(pbpt::math::normalize(orientation));
        m_transform_dirty = true;
    }

    pbpt::math::Vec3 angular_velocity() const { return m_source_body.state().rotation.angular_velocity; }
    void set_angular_velocity(const pbpt::math::Vec3& angular_velocity) {
        m_source_body.state().rotation.angular_velocity = angular_velocity;
        m_source_dirty = true;
    }

    pbpt::math::Float mass() const { return m_source_body.state().mass; }
    void set_mass(pbpt::math::Float mass) {
        m_source_body.state().mass = sanitize_mass(mass);
        m_source_dirty = true;
    }

    bool use_gravity() const { return m_source_body.use_gravity(); }
    void set_use_gravity(bool use_gravity) {
        m_source_body.set_use_gravity(use_gravity);
        m_source_dirty = true;
    }

    pbpt::math::Float restitution() const { return m_source_body.restitution(); }
    void set_restitution(pbpt::math::Float restitution) {
        m_source_body.set_restitution(sanitize_restitution(restitution));
        m_source_dirty = true;
    }

    pbpt::math::Float friction() const { return m_source_body.friction(); }
    void set_friction(pbpt::math::Float friction) {
        m_source_body.set_friction(sanitize_friction(friction));
        m_source_dirty = true;
    }

    pbpt::math::Float linear_decay() const { return m_source_body.linear_decay(); }
    void set_linear_decay(pbpt::math::Float linear_decay) {
        m_source_body.set_linear_decay(sanitize_decay(linear_decay, "RigidBody linear_decay"));
        m_source_dirty = true;
    }

    pbpt::math::Float angular_decay() const { return m_source_body.angular_decay(); }
    void set_angular_decay(pbpt::math::Float angular_decay) {
        m_source_body.set_angular_decay(sanitize_decay(angular_decay, "RigidBody angular_decay"));
        m_source_dirty = true;
    }

    pbpt::math::Mat3 inverse_inertia_tensor_ref() const { return m_source_body.inverse_inertia_tensor_ref(); }
    void set_inverse_inertia_tensor_ref(const pbpt::math::Mat3& inverse_inertia_tensor_ref) {
        m_source_body.set_inverse_inertia_tensor_ref(
            sanitize_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref)
        );
        m_source_dirty = true;
    }

    void add_force(const pbpt::math::Vec3& force) {
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::AddForce,
            .value = force,
        });
    }

    void add_torque(const pbpt::math::Vec3& torque) {
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::AddTorque,
            .value = torque,
        });
    }

    void add_force_at_point(const pbpt::math::Vec3& force, const pbpt::math::Vec3& world_point) {
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::AddForceAtPoint,
            .value = force,
            .point = world_point,
        });
    }

    void clear_forces() {
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::ClearForces,
        });
    }

    void reset_dynamics() {
        m_source_body.reset_dynamics();
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::ResetDynamics,
        });
    }

    void reset_translation_dynamics() {
        m_source_body.reset_translation_dynamics();
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::ResetTranslationDynamics,
        });
    }

    void reset_rotational_dynamics() {
        m_source_body.reset_rotational_dynamics();
        enqueue_command(PendingRigidBodyCommand{
            .type = PendingRigidBodyCommandType::ResetRotationalDynamics,
        });
    }
};

}  // namespace rtr::framework::component
