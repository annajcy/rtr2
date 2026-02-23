#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/camera_controller.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

struct FreeLookCameraControllerConfig {
    float move_speed{5.0f};
    float sprint_multiplier{3.0f};
    float mouse_sensitivity{0.12f};  // degree per pixel
    float zoom_speed{0.8f};
    float pitch_min_degrees{-89.0f};
    float pitch_max_degrees{89.0f};
};

class FreeLookCameraController final : public CameraController {
private:
    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("framework.component.free_look"); }

    static constexpr float kEpsilon = 1e-5f;

    FreeLookCameraControllerConfig   m_config{};

    bool  m_angles_initialized{false};
    float m_yaw_degrees{0.0f};
    float m_pitch_degrees{0.0f};

    void validate_config(const FreeLookCameraControllerConfig& config) const {
        if (config.pitch_min_degrees > config.pitch_max_degrees) {
            logger()->error("FreeLook config invalid: pitch_min_degrees={} > pitch_max_degrees={}.",
                            config.pitch_min_degrees, config.pitch_max_degrees);
            throw std::invalid_argument("FreeLookCameraControllerConfig pitch_min_degrees must be <= pitch_max_degrees.");
        }
    }

    void validate_controller_config() const override {
        validate_config(m_config);
    }

    void initialize_angles_from_front() {
        const pbpt::math::vec3 front = pbpt::math::normalize(require_camera_component().camera_world_front());
        m_yaw_degrees                = pbpt::math::degrees(std::atan2(front.x(), front.z()));
        m_pitch_degrees              = pbpt::math::degrees(std::asin(pbpt::math::clamp(front.y(), -1.0f, 1.0f)));
        m_pitch_degrees              = pbpt::math::clamp(m_pitch_degrees, m_config.pitch_min_degrees, m_config.pitch_max_degrees);
        m_angles_initialized         = true;
    }

    pbpt::math::quat world_rotation_looking_to(const pbpt::math::vec3& forward_dir) const {
        const pbpt::math::vec3 forward = pbpt::math::normalize(forward_dir);
        pbpt::math::vec3       up      = pbpt::math::vec3(0.0f, 1.0f, 0.0f);
        if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
            up = pbpt::math::vec3(0.0f, 0.0f, 1.0f);
            if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
                up = pbpt::math::vec3(1.0f, 0.0f, 0.0f);
            }
        }

        const pbpt::math::vec3 right        = pbpt::math::normalize(pbpt::math::cross(forward, up));
        const pbpt::math::vec3 corrected_up = pbpt::math::normalize(pbpt::math::cross(right, forward));

        // Camera convention: local -Z is front.
        const pbpt::math::mat3 basis = pbpt::math::mat3::from_cols(right, corrected_up, -forward);
        return pbpt::math::normalize(pbpt::math::quat_cast(basis));
    }

public:
    explicit FreeLookCameraController(core::GameObject& owner,
                                      const system::input::InputState& input_state,
                                      FreeLookCameraControllerConfig   config = {})
        : CameraController(owner, input_state), m_config(config) {
        validate_config(m_config);
    }

    void set_config(const FreeLookCameraControllerConfig& config) {
        validate_config(config);
        m_config = config;
    }

    const FreeLookCameraControllerConfig& config() const { return m_config; }

    void on_update_active_camera(const core::FrameTickContext& ctx, Camera& camera) override {
        auto& go = require_owner();
        if (!m_angles_initialized) {
            initialize_angles_from_front();
        }

        auto node = go.node();
        const auto& input = input_state();
        if (input.mouse_button_down(system::input::MouseButton::RIGHT)) {
            m_yaw_degrees += static_cast<float>(input.mouse_dx()) * m_config.mouse_sensitivity;
            m_pitch_degrees -= static_cast<float>(input.mouse_dy()) * m_config.mouse_sensitivity;
            m_pitch_degrees = pbpt::math::clamp(m_pitch_degrees, m_config.pitch_min_degrees, m_config.pitch_max_degrees);

            const float yaw_rad   = pbpt::math::radians(m_yaw_degrees);
            const float pitch_rad = pbpt::math::radians(m_pitch_degrees);
            const float cos_pitch = std::cos(pitch_rad);
            const pbpt::math::vec3 desired_front = pbpt::math::normalize(pbpt::math::vec3{
                std::sin(yaw_rad) * cos_pitch,
                std::sin(pitch_rad),
                std::cos(yaw_rad) * cos_pitch});
            node.set_world_rotation(world_rotation_looking_to(desired_front));
        }

        float speed = m_config.move_speed;
        if (input.key_down(system::input::KeyCode::LEFT_SHIFT)) {
            speed *= m_config.sprint_multiplier;
        }

        pbpt::math::vec3 move_direction(0.0f);
        const pbpt::math::vec3 world_front = camera.camera_world_front();
        const pbpt::math::vec3 world_right = node.world_right();
        const pbpt::math::vec3 world_up    = node.world_up();

        if (input.key_down(system::input::KeyCode::W))
            move_direction += world_front;
        if (input.key_down(system::input::KeyCode::S))
            move_direction -= world_front;
        if (input.key_down(system::input::KeyCode::D))
            move_direction += world_right;
        if (input.key_down(system::input::KeyCode::A))
            move_direction -= world_right;
        if (input.key_down(system::input::KeyCode::E))
            move_direction += world_up;
        if (input.key_down(system::input::KeyCode::Q))
            move_direction -= world_up;

        if (pbpt::math::length(move_direction) > 0.0f) {
            move_direction              = pbpt::math::normalize(move_direction);
            const float           dt    = static_cast<float>(std::max(ctx.delta_seconds, 0.0));
            const pbpt::math::vec3 delta = move_direction * speed * dt;
            const pbpt::math::vec3 new_position = node.world_position() + delta;
            node.set_world_position(new_position);
            logger()->trace("FreeLook node position updated (game_object_id={}, position=[{:.4f}, {:.4f}, {:.4f}]).",
                            go.id(), new_position.x(), new_position.y(), new_position.z());
        }

        const float scroll_y = static_cast<float>(input.mouse_scroll_dy());
        if (scroll_y != 0.0f) {
            camera.adjust_zoom(scroll_y * m_config.zoom_speed);
        }
    }
};

}  // namespace rtr::framework::component
