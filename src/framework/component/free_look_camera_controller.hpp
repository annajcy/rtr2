#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/trigonometric.hpp>

#include "framework/component/component.hpp"
#include "framework/core/camera_manager.hpp"
#include "framework/core/game_object.hpp"
#include "system/input/input_state.hpp"
#include "system/input/input_types.hpp"

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
    const system::input::InputState* m_input_state{nullptr};
    core::CameraManager* m_camera_manager{nullptr};
    FreeLookCameraControllerConfig m_config{};

    bool m_validated_once{false};
    bool m_angles_initialized{false};
    float m_yaw_degrees{0.0f};
    float m_pitch_degrees{0.0f};

    void validate_config(const FreeLookCameraControllerConfig& config) const {
        if (config.pitch_min_degrees > config.pitch_max_degrees) {
            throw std::invalid_argument(
                "FreeLookCameraControllerConfig pitch_min_degrees must be <= pitch_max_degrees."
            );
        }
    }

    void validate_dependencies() {
        if (owner() == nullptr) {
            throw std::runtime_error("FreeLookCameraController owner is null.");
        }
        if (m_input_state == nullptr) {
            throw std::runtime_error("FreeLookCameraController input_state is null.");
        }
        if (m_camera_manager == nullptr) {
            throw std::runtime_error("FreeLookCameraController camera_manager is null.");
        }
        if (m_camera_manager->camera(owner()->id()) == nullptr) {
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
        const auto node = owner()->node();
        const glm::vec3 front = glm::normalize(node.world_front());
        m_yaw_degrees = glm::degrees(std::atan2(front.x, front.z));
        m_pitch_degrees = glm::degrees(std::asin(glm::clamp(front.y, -1.0f, 1.0f)));
        m_pitch_degrees = glm::clamp(
            m_pitch_degrees,
            m_config.pitch_min_degrees,
            m_config.pitch_max_degrees
        );
        m_angles_initialized = true;
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
            m_pitch_degrees = glm::clamp(
                m_pitch_degrees,
                m_config.pitch_min_degrees,
                m_config.pitch_max_degrees
            );

            const glm::quat yaw_q = glm::angleAxis(
                glm::radians(m_yaw_degrees),
                glm::vec3(0.0f, 1.0f, 0.0f)
            );
            const glm::quat pitch_q = glm::angleAxis(
                glm::radians(m_pitch_degrees),
                glm::vec3(1.0f, 0.0f, 0.0f)
            );
            node.set_local_rotation(glm::normalize(yaw_q * pitch_q));
        }

        float speed = m_config.move_speed;
        if (m_input_state->key_down(system::input::KeyCode::LEFT_SHIFT)) {
            speed *= m_config.sprint_multiplier;
        }

        glm::vec3 move_direction(0.0f);
        const glm::vec3 world_front = node.world_front();
        const glm::vec3 world_right = node.world_right();
        const glm::vec3 world_up = node.world_up();

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

        if (glm::length(move_direction) > 0.0f) {
            move_direction = glm::normalize(move_direction);
            const float dt = static_cast<float>(std::max(ctx.delta_seconds, 0.0));
            node.set_world_position(node.world_position() + move_direction * speed * dt);
        }

        const float scroll_y = static_cast<float>(m_input_state->mouse_scroll_dy());
        if (scroll_y != 0.0f) {
            auto* camera = m_camera_manager->camera(go->id());
            if (camera == nullptr) {
                throw std::runtime_error(
                    "FreeLookCameraController owner does not have a bound camera."
                );
            }
            camera->adjust_zoom(scroll_y * m_config.zoom_speed);
        }
    }
};

} // namespace rtr::framework::component
