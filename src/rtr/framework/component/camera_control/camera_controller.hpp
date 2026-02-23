#pragma once

#include <functional>
#include <stdexcept>

#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::component {

class CameraController : public Component {
private:
    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("framework.component.camera_controller");
    }

    std::reference_wrapper<const system::input::InputState> m_input_state;
    bool                                                     m_validated_once{false};

    void validate_dependencies() {
        (void)require_camera_component();
        validate_controller_config();
        m_validated_once = true;
    }

    void ensure_validated() {
        if (!m_validated_once) {
            validate_dependencies();
        }
    }

protected:
    explicit CameraController(core::GameObject& owner, const system::input::InputState& input_state)
        : Component(owner), m_input_state(input_state) {}

    virtual void validate_controller_config() const {}

    core::GameObject& require_owner() {
        return owner();
    }

    const core::GameObject& require_owner() const {
        return owner();
    }

    Camera& require_camera_component() {
        return require_owner().component_or_throw<Camera>();
    }

    const Camera& require_camera_component() const {
        return require_owner().component_or_throw<Camera>();
    }

    const system::input::InputState& input_state() const {
        return m_input_state.get();
    }

    virtual void on_update_active_camera(const core::FrameTickContext& ctx, Camera& camera) = 0;

public:
    void on_awake() final { validate_dependencies(); }

    void on_update(const core::FrameTickContext& ctx) final {
        ensure_validated();
        auto& camera = require_camera_component();
        if (!camera.active()) {
            return;
        }
        on_update_active_camera(ctx, camera);
    }
};

}  // namespace rtr::framework::component
