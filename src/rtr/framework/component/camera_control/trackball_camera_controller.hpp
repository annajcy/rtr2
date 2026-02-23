#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

struct TrackBallCameraControllerConfig {
    float            rotate_speed{0.06f};  // degree per pixel
    float            pan_speed{0.0012f};   // world unit / (pixel * distance)
    float            zoom_speed{0.35f};
    float            pitch_min_degrees{-89.0f};
    float            pitch_max_degrees{89.0f};
    pbpt::math::vec3 world_up{0.0f, 1.0f, 0.0f};
    pbpt::math::vec3 default_target{0.0f, 0.0f, 0.0f};
};

class TrackBallCameraController final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("framework.component.trackball"); }

    static constexpr float kEpsilon = 1e-5f;

    const system::input::InputState* m_input_state{nullptr};
    TrackBallCameraControllerConfig  m_config{};

    bool m_validated_once{false};
    bool m_orbit_initialized{false};

    pbpt::math::vec3 m_target_world{0.0f, 0.0f, 0.0f};
    float            m_radius{1.0f};
    float            m_yaw_degrees{0.0f};
    float            m_pitch_degrees{0.0f};

    void validate_config(const TrackBallCameraControllerConfig& config) const {
        if (config.pitch_min_degrees > config.pitch_max_degrees) {
            logger()->error("TrackBall config invalid: pitch_min_degrees={} > pitch_max_degrees={}.",
                            config.pitch_min_degrees, config.pitch_max_degrees);
            throw std::invalid_argument("TrackBallCameraControllerConfig pitch_min_degrees must be <= pitch_max_degrees.");
        }
        if (pbpt::math::length(config.world_up) <= kEpsilon) {
            logger()->error("TrackBall config invalid: world_up has zero length.");
            throw std::invalid_argument("TrackBallCameraControllerConfig world_up must have non-zero length.");
        }
    }

    Camera& require_camera_component() {
        auto* go = owner();
        if (go == nullptr) {
            logger()->error("TrackBall camera lookup failed: owner is null.");
            throw std::runtime_error("TrackBallCameraController owner is null.");
        }
        auto* camera = go->get_component<Camera>();
        if (camera == nullptr) {
            logger()->error("TrackBall camera lookup failed: owner {} has no Camera.", go->id());
            throw std::runtime_error("TrackBallCameraController owner does not have a Camera.");
        }
        return *camera;
    }

    void validate_dependencies() {
        if (owner() == nullptr) {
            logger()->error("TrackBall validate_dependencies failed: owner is null.");
            throw std::runtime_error("TrackBallCameraController owner is null.");
        }
        if (m_input_state == nullptr) {
            logger()->error("TrackBall validate_dependencies failed: input_state is null.");
            throw std::runtime_error("TrackBallCameraController input_state is null.");
        }
        (void)require_camera_component();
        m_validated_once = true;
    }

    void ensure_validated() {
        if (!m_validated_once) {
            validate_dependencies();
        }
    }

    void sync_spherical_from_current_position() {
        const auto node                     = owner()->node();
        const pbpt::math::vec3 world_pos    = node.world_position();
        const pbpt::math::vec3 offset       = world_pos - m_target_world;
        m_radius                            = std::max(pbpt::math::length(offset), kEpsilon);
        m_yaw_degrees                       = pbpt::math::degrees(std::atan2(offset.x(), offset.z()));
        m_pitch_degrees                     = pbpt::math::degrees(pbpt::math::asin(pbpt::math::clamp(offset.y() / m_radius, -1.0f, 1.0f)));
        m_pitch_degrees                     = pbpt::math::clamp(m_pitch_degrees, m_config.pitch_min_degrees, m_config.pitch_max_degrees);
    }

    void initialize_orbit_state() {
        sync_spherical_from_current_position();
        auto node = owner()->node();
        const pbpt::math::vec3 look_dir = m_target_world - node.world_position();
        if (pbpt::math::length(look_dir) > kEpsilon) {
            node.set_world_rotation(world_rotation_looking_to(look_dir));
        }
        m_orbit_initialized = true;
    }

    pbpt::math::vec3 spherical_direction() const {
        const float yaw_rad   = pbpt::math::radians(m_yaw_degrees);
        const float pitch_rad = pbpt::math::radians(m_pitch_degrees);
        const float cos_pitch = std::cos(pitch_rad);
        return pbpt::math::normalize(
            pbpt::math::vec3{std::sin(yaw_rad) * cos_pitch, std::sin(pitch_rad), std::cos(yaw_rad) * cos_pitch});
    }

    pbpt::math::quat world_rotation_looking_to(const pbpt::math::vec3& forward_dir) const {
        const pbpt::math::vec3 forward = pbpt::math::normalize(forward_dir);
        pbpt::math::vec3       up      = pbpt::math::normalize(m_config.world_up);
        if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
            up = pbpt::math::vec3(0.0f, 0.0f, 1.0f);
            if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
                up = pbpt::math::vec3(1.0f, 0.0f, 0.0f);
            }
        }

        const pbpt::math::vec3 right        = pbpt::math::normalize(pbpt::math::cross(forward, up));
        const pbpt::math::vec3 corrected_up = pbpt::math::normalize(pbpt::math::cross(right, forward));
        const pbpt::math::mat3 basis        = pbpt::math::mat3::from_cols(right, corrected_up, -forward);
        return pbpt::math::normalize(pbpt::math::quat_cast(basis));
    }

    void apply_pose_from_orbit_state() {
        auto node = owner()->node();
        const pbpt::math::vec3 dir      = spherical_direction();
        const pbpt::math::vec3 position = m_target_world + dir * m_radius;
        const pbpt::math::vec3 look_dir = m_target_world - position;
        if (pbpt::math::length(look_dir) <= kEpsilon) {
            return;
        }
        node.set_world_position(position);
        node.set_world_rotation(world_rotation_looking_to(look_dir));
        logger()->trace(
            "TrackBall node orbit updated (game_object_id={}, target=[{:.4f}, {:.4f}, {:.4f}], radius={:.4f}).",
            owner()->id(), m_target_world.x(), m_target_world.y(), m_target_world.z(), m_radius);
    }

public:
    explicit TrackBallCameraController(const system::input::InputState* input_state,
                                       TrackBallCameraControllerConfig   config = {})
        : m_input_state(input_state), m_config(config), m_target_world(config.default_target) {
        validate_config(m_config);
    }

    void set_input_state(const system::input::InputState* input_state) {
        m_input_state    = input_state;
        m_validated_once = false;
    }

    void set_config(const TrackBallCameraControllerConfig& config) {
        validate_config(config);
        m_config = config;
    }

    const TrackBallCameraControllerConfig& config() const { return m_config; }

    void set_target(const pbpt::math::vec3& target_world) {
        m_target_world      = target_world;
        m_orbit_initialized = false;
    }

    const pbpt::math::vec3& target() const { return m_target_world; }

    void on_awake() override { validate_dependencies(); }

    void on_update(const core::FrameTickContext& /*ctx*/) override {
        ensure_validated();

        auto* go = owner();
        if (go == nullptr) {
            logger()->error("TrackBall on_update failed: owner is null.");
            throw std::runtime_error("TrackBallCameraController owner is null.");
        }
        auto& camera = require_camera_component();
        if (!camera.active()) {
            return;
        }
        if (!m_orbit_initialized) {
            initialize_orbit_state();
        }

        const bool left_down   = m_input_state->mouse_button_down(system::input::MouseButton::LEFT);
        const bool middle_down = m_input_state->mouse_button_down(system::input::MouseButton::MIDDLE);

        if (left_down) {
            m_yaw_degrees += static_cast<float>(m_input_state->mouse_dx()) * m_config.rotate_speed;
            m_pitch_degrees += static_cast<float>(m_input_state->mouse_dy()) * m_config.rotate_speed;
            m_pitch_degrees = pbpt::math::clamp(m_pitch_degrees, m_config.pitch_min_degrees, m_config.pitch_max_degrees);
            apply_pose_from_orbit_state();
        } else if (middle_down) {
            auto node = go->node();
            const float distance_scale = std::max(m_radius, kEpsilon);
            const pbpt::math::vec3 delta =
                node.world_right() * static_cast<float>(m_input_state->mouse_dx()) * m_config.pan_speed * distance_scale +
                node.world_up() * static_cast<float>(m_input_state->mouse_dy()) * m_config.pan_speed * distance_scale;

            m_target_world += delta;
            node.set_world_position(node.world_position() + delta);
            const pbpt::math::vec3 look_dir = m_target_world - node.world_position();
            if (pbpt::math::length(look_dir) > kEpsilon) {
                node.set_world_rotation(world_rotation_looking_to(look_dir));
            }
        }

        const float scroll_y = static_cast<float>(m_input_state->mouse_scroll_dy());
        if (scroll_y != 0.0f) {
            camera.adjust_zoom(scroll_y * m_config.zoom_speed);
            if (dynamic_cast<PerspectiveCamera*>(&camera) != nullptr) {
                sync_spherical_from_current_position();
            }
        }
    }
};

}  // namespace rtr::framework::component
