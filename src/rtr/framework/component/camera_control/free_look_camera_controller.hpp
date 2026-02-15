#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>


#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/camera_manager.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

struct FreeLookCameraControllerConfig {
    float move_speed{5.0f};
    float sprint_multiplier{3.0f};
    float mouse_sensitivity{0.12f}; // degree per pixel
    float zoom_speed{0.8f};
    float pitch_min_degrees{-89.0f};
    float pitch_max_degrees{89.0f};
};

class FreeLookCameraController final : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.free_look");
    }

    static constexpr float kEpsilon = 1e-5f;

    const system::input::InputState* m_input_state{nullptr};
    core::CameraManager* m_camera_manager{nullptr};
    FreeLookCameraControllerConfig m_config{};

    bool m_validated_once{false};
    bool m_angles_initialized{false};
    float m_yaw_degrees{0.0f};
    float m_pitch_degrees{0.0f};

    void validate_config(const FreeLookCameraControllerConfig& config) const {
        if (config.pitch_min_degrees > config.pitch_max_degrees) {
            logger()->error(
                "FreeLook config invalid: pitch_min_degrees={} > pitch_max_degrees={}.",
                config.pitch_min_degrees,
                config.pitch_max_degrees
            );
            throw std::invalid_argument(
                "FreeLookCameraControllerConfig pitch_min_degrees must be <= pitch_max_degrees."
            );
        }
    }

    void validate_dependencies() {
        if (owner() == nullptr) {
            logger()->error("FreeLook validate_dependencies failed: owner is null.");
            throw std::runtime_error("FreeLookCameraController owner is null.");
        }
        if (m_input_state == nullptr) {
            logger()->error("FreeLook validate_dependencies failed: input_state is null.");
            throw std::runtime_error("FreeLookCameraController input_state is null.");
        }
        if (m_camera_manager == nullptr) {
            logger()->error("FreeLook validate_dependencies failed: camera_manager is null.");
            throw std::runtime_error("FreeLookCameraController camera_manager is null.");
        }
        if (m_camera_manager->camera(owner()->id()) == nullptr) {
            logger()->error(
                "FreeLook validate_dependencies failed: no camera bound for owner {}.",
                owner()->id()
            );
            throw std::runtime_error(
                "FreeLookCameraController owner does not have a bound camera."
            );
        }
        m_validated_once = true;
    }

    void ensure_validated() {
        if (!m_validated_once) {
            validate_dependencies();
        }
    }

    void initialize_angles_from_front() {
        auto* camera = m_camera_manager->camera(owner()->id());
        if (camera == nullptr) {
            logger()->error(
                "FreeLook initialize_angles_from_front failed: no camera bound for owner {}.",
                owner()->id()
            );
            throw std::runtime_error(
                "FreeLookCameraController owner does not have a bound camera."
            );
        }

        const pbpt::math::vec3 front = pbpt::math::normalize(camera->front());
        m_yaw_degrees = pbpt::math::degrees(std::atan2(front.x(), front.z()));
        m_pitch_degrees = pbpt::math::degrees(std::asin(pbpt::math::clamp(front.y(), -1.0f, 1.0f)));
        m_pitch_degrees = pbpt::math::clamp(
            m_pitch_degrees,
            m_config.pitch_min_degrees,
            m_config.pitch_max_degrees
        );
        m_angles_initialized = true;
    }

    pbpt::math::quat world_rotation_looking_to(const pbpt::math::vec3& forward_dir) const {
        const pbpt::math::vec3 forward = pbpt::math::normalize(forward_dir);
        pbpt::math::vec3 up = pbpt::math::vec3(0.0f, 1.0f, 0.0f);
        if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
            up = pbpt::math::vec3(0.0f, 0.0f, 1.0f);
            if (pbpt::math::length(pbpt::math::cross(up, forward)) <= kEpsilon) {
                up = pbpt::math::vec3(1.0f, 0.0f, 0.0f);
            }
        }

        const pbpt::math::vec3 right = pbpt::math::normalize(pbpt::math::cross(forward, up));
        const pbpt::math::vec3 corrected_up = pbpt::math::normalize(pbpt::math::cross(right, forward));

        // Camera convention: local -Z is front.
        const pbpt::math::mat3 basis = pbpt::math::mat3::from_cols(right, corrected_up, -forward);
        return pbpt::math::normalize(pbpt::math::quat_cast(basis));
    }

public:
    explicit FreeLookCameraController(
        const system::input::InputState* input_state,
        core::CameraManager* camera_manager,
        FreeLookCameraControllerConfig config = {}
    )
        : m_input_state(input_state), m_camera_manager(camera_manager), m_config(config) {
        validate_config(m_config);
    }

    void set_input_state(const system::input::InputState* input_state) {
        m_input_state = input_state;
        m_validated_once = false;
    }

    void set_camera_manager(core::CameraManager* camera_manager) {
        m_camera_manager = camera_manager;
        m_validated_once = false;
    }

    void set_config(const FreeLookCameraControllerConfig& config) {
        validate_config(config);
        m_config = config;
    }

    const FreeLookCameraControllerConfig& config() const {
        return m_config;
    }

    void on_awake() override {
        validate_dependencies();
    }

    void on_update(const core::FrameTickContext& ctx) override {
        ensure_validated();

        auto* go = owner();
        if (go == nullptr) {
            logger()->error("FreeLook on_update failed: owner is null.");
            throw std::runtime_error("FreeLookCameraController owner is null.");
        }

        if (m_camera_manager->active_camera_owner_id() != go->id()) {
            return;
        }

        if (!m_angles_initialized) {
            initialize_angles_from_front();
        }

        auto node = go->node();

        if (m_input_state->mouse_button_down(system::input::MouseButton::RIGHT)) {
            m_yaw_degrees += static_cast<float>(m_input_state->mouse_dx()) * m_config.mouse_sensitivity;
            m_pitch_degrees -= static_cast<float>(m_input_state->mouse_dy()) * m_config.mouse_sensitivity;
            m_pitch_degrees = pbpt::math::clamp(
                m_pitch_degrees,
                m_config.pitch_min_degrees,
                m_config.pitch_max_degrees
            );

            const float yaw_rad = pbpt::math::radians(m_yaw_degrees);
            const float pitch_rad = pbpt::math::radians(m_pitch_degrees);
            const float cos_pitch = std::cos(pitch_rad);
            const pbpt::math::vec3 desired_front = pbpt::math::normalize(pbpt::math::vec3{
                std::sin(yaw_rad) * cos_pitch,
                std::sin(pitch_rad),
                std::cos(yaw_rad) * cos_pitch
            });
            node.set_world_rotation(world_rotation_looking_to(desired_front));
            logger()->trace(
                "FreeLook node rotation updated (owner_id={}, yaw_deg={}, pitch_deg={})",
                go->id(),
                m_yaw_degrees,
                m_pitch_degrees
            );
        }

        float speed = m_config.move_speed;
        if (m_input_state->key_down(system::input::KeyCode::LEFT_SHIFT)) {
            speed *= m_config.sprint_multiplier;
        }

        pbpt::math::vec3 move_direction(0.0f);
        auto* camera = m_camera_manager->camera(go->id());
        if (camera == nullptr) {
            logger()->error(
                "FreeLook on_update failed: no camera bound for owner {}.",
                go->id()
            );
            throw std::runtime_error(
                "FreeLookCameraController owner does not have a bound camera."
            );
        }
        const pbpt::math::vec3 world_front = camera->front();
        const pbpt::math::vec3 world_right = node.world_right();
        const pbpt::math::vec3 world_up = node.world_up();

        if (m_input_state->key_down(system::input::KeyCode::W)) {
            move_direction += world_front;
        }
        if (m_input_state->key_down(system::input::KeyCode::S)) {
            move_direction -= world_front;
        }
        if (m_input_state->key_down(system::input::KeyCode::D)) {
            move_direction += world_right;
        }
        if (m_input_state->key_down(system::input::KeyCode::A)) {
            move_direction -= world_right;
        }
        if (m_input_state->key_down(system::input::KeyCode::E)) {
            move_direction += world_up;
        }
        if (m_input_state->key_down(system::input::KeyCode::Q)) {
            move_direction -= world_up;
        }

        if (pbpt::math::length(move_direction) > 0.0f) {
            move_direction = pbpt::math::normalize(move_direction);
            const float dt = static_cast<float>(std::max(ctx.delta_seconds, 0.0));
            const pbpt::math::vec3 delta = move_direction * speed * dt;
            node.set_world_position(node.world_position() + delta);
            logger()->trace(
                "FreeLook node position updated (owner_id={}, delta=[{:.4f}, {:.4f}, {:.4f}], speed={:.4f}, dt={:.4f})",
                go->id(),
                delta.x(),
                delta.y(),
                delta.z(),
                speed,
                dt
            );
        }

        const float scroll_y = static_cast<float>(m_input_state->mouse_scroll_dy());
        if (scroll_y != 0.0f) {
            camera->adjust_zoom(scroll_y * m_config.zoom_speed);
        }
    }
};

} // namespace rtr::framework::component
