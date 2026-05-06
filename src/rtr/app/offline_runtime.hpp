#pragma once

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/app/frame_stepper.hpp"
#include "rtr/app/frame_time_policy.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/frame_constants.hpp"
#include "rtr/rhi/window.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/physics/physics_system.hpp"
#include "rtr/system/render/output_backend.hpp"
#include "rtr/system/render/render_pipeline.hpp"
#include "rtr/system/render/renderer.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::app {

struct OfflineRuntimeConfig {
    uint32_t width{1920};
    uint32_t height{1080};

    double sim_dt{0.01};
    uint32_t steps_per_output_frame{1};
    uint32_t total_output_frames{500};
    double output_fps{100.0};

    bool export_frames{true};
    std::filesystem::path output_dir{"output/frames"};
    std::string filename_pattern{"frame_{:06d}.png"};

    bool create_non_interactive_window{true};
    bool warn_on_fps_mismatch{true};
    bool fail_on_fps_mismatch{false};
};

struct OfflineRuntimeResult {
    bool ok{true};
    std::string error_message{};
    std::uint64_t frames_rendered{0};
    std::uint64_t fixed_ticks{0};
};

struct OfflineRuntimeContext {
    framework::core::World& world;
    resource::ResourceManager& resources;
    system::render::IRenderer& renderer;
    system::physics::PhysicsSystem& physics_system;
    std::uint64_t frame_serial{0};
    std::uint32_t output_frame_index{0};
    double delta_seconds{0.0};
    std::function<void()> request_stop{};
};

struct OfflineRuntimeCallbacks {
    std::function<void(OfflineRuntimeContext&)> on_startup{};
    std::function<void(OfflineRuntimeContext&)> on_pre_update{};
    std::function<void(OfflineRuntimeContext&)> on_post_update{};
    std::function<void(OfflineRuntimeContext&)> on_pre_render{};
    std::function<void(OfflineRuntimeContext&)> on_post_render{};
    std::function<void(OfflineRuntimeContext&)> on_shutdown{};
};

inline void validate_offline_runtime_config(const OfflineRuntimeConfig& config) {
    auto log = utils::get_logger("app.offline_runtime");

    if (config.width == 0 || config.height == 0) {
        throw std::invalid_argument("OfflineRuntimeConfig width/height must be positive.");
    }
    if (config.sim_dt <= 0.0) {
        throw std::invalid_argument("OfflineRuntimeConfig sim_dt must be positive.");
    }
    if (config.steps_per_output_frame == 0) {
        throw std::invalid_argument("OfflineRuntimeConfig steps_per_output_frame must be positive.");
    }
    if (config.total_output_frames == 0) {
        throw std::invalid_argument("OfflineRuntimeConfig total_output_frames must be positive.");
    }
    if (config.output_fps <= 0.0) {
        throw std::invalid_argument("OfflineRuntimeConfig output_fps must be positive.");
    }
    if (config.export_frames && config.output_dir.empty()) {
        throw std::invalid_argument("OfflineRuntimeConfig output_dir must not be empty.");
    }

    const double expected_fps =
        1.0 / (config.sim_dt * static_cast<double>(config.steps_per_output_frame));
    if (std::abs(config.output_fps - expected_fps) > 1e-6) {
        if (config.warn_on_fps_mismatch) {
            log->warn(
                "OfflineRuntime output_fps ({}) does not match simulation fps ({}).",
                config.output_fps,
                expected_fps
            );
        }
        if (config.fail_on_fps_mismatch) {
            throw std::invalid_argument("OfflineRuntimeConfig output_fps does not match sim_dt * steps_per_output_frame.");
        }
    }
}

class OfflineRuntime {
private:
    static constexpr const char* kDefaultWindowTitle = "RTR Headless Offline Runtime";
    static constexpr const char* kDefaultResourceRootDir = "./assets/";

    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("app.offline_runtime");
    }

    static rhi::WindowCreateInfo make_window_create_info(const OfflineRuntimeConfig& config) {
        return rhi::WindowCreateInfo{
            .width = static_cast<int>(config.width),
            .height = static_cast<int>(config.height),
            .title = kDefaultWindowTitle,
            .resizable = false,
            .visible = true,
            .focus_on_show = !config.create_non_interactive_window,
        };
    }

    static system::render::OfflineImageOutputConfig make_backend_config(const OfflineRuntimeConfig& config) {
        return system::render::OfflineImageOutputConfig{
            .export_frames = config.export_frames,
            .output_dir = config.output_dir,
            .filename_pattern = config.filename_pattern,
            .starting_output_frame_index = 0
        };
    }

    OfflineRuntimeConfig m_config{};
    OfflineRuntimeCallbacks m_callbacks{};

    resource::ResourceManager m_resources;
    system::render::RendererT<system::render::OfflineImageOutputBackend> m_renderer;
    system::input::InputSystem m_input_system;
    system::physics::PhysicsSystem m_physics_system;
    framework::core::World m_world;

    bool m_stop_requested{false};
    std::uint64_t m_frame_serial{0};
    std::uint64_t m_fixed_tick_serial{0};
    FrameStepper m_frame_stepper{};
    OfflineFrameTimePolicy m_time_policy;

public:
    explicit OfflineRuntime(OfflineRuntimeConfig config = {})
        : m_config(std::move(config)),
          m_resources(kDefaultResourceRootDir),
          m_renderer(make_window_create_info(m_config), make_backend_config(m_config)),
          m_input_system(system::input::InputSystem::RawEventSource{}),
          m_physics_system(),
          m_world(m_resources),
          m_time_policy(m_config.sim_dt, m_config.steps_per_output_frame) {
        logger()->info(
            "OfflineRuntime initialized (window={}x{}, total_output_frames={}, sim_dt={}, steps_per_output_frame={})",
            m_config.width,
            m_config.height,
            m_config.total_output_frames,
            m_config.sim_dt,
            m_config.steps_per_output_frame
        );
    }

    const OfflineRuntimeConfig& config() const { return m_config; }

    framework::core::World& world() { return m_world; }
    const framework::core::World& world() const { return m_world; }

    resource::ResourceManager& resource_manager() { return m_resources; }
    const resource::ResourceManager& resource_manager() const { return m_resources; }

    system::render::RendererT<system::render::OfflineImageOutputBackend>& renderer() { return m_renderer; }
    const system::render::RendererT<system::render::OfflineImageOutputBackend>& renderer() const { return m_renderer; }

    system::physics::PhysicsSystem& physics_system() { return m_physics_system; }
    const system::physics::PhysicsSystem& physics_system() const { return m_physics_system; }

    void set_callbacks(OfflineRuntimeCallbacks callbacks) { m_callbacks = std::move(callbacks); }
    const OfflineRuntimeCallbacks& callbacks() const { return m_callbacks; }

    void set_pipeline(std::unique_ptr<system::render::RenderPipeline> pipeline) {
        if (!pipeline) {
            throw std::runtime_error("OfflineRuntime set_pipeline received null pipeline.");
        }
        m_renderer.set_pipeline(std::move(pipeline));
        logger()->info("Pipeline bound to offline runtime.");
    }

    void request_stop() { m_stop_requested = true; }
    bool stop_requested() const { return m_stop_requested; }

    OfflineRuntimeResult run() {
        auto log = logger();
        OfflineRuntimeResult result{};

        if (m_renderer.pipeline() == nullptr) {
            result.ok = false;
            result.error_message = "OfflineRuntime requires pipeline before run().";
            log->error("run() aborted: pipeline is not bound.");
            return result;
        }

        try {
            validate_offline_runtime_config(m_config);
            if (m_config.export_frames) {
                std::filesystem::create_directories(m_config.output_dir);
            }

            if (m_callbacks.on_startup) {
                auto ctx = make_runtime_context(0.0, 0);
                m_callbacks.on_startup(ctx);
            }

            for (std::uint32_t output_frame_index = 0;
                 output_frame_index < m_config.total_output_frames && !m_stop_requested;
                 ++output_frame_index) {
                const FrameExecutionPlan frame_plan = m_time_policy.make_plan(false);

                if (m_callbacks.on_pre_update) {
                    auto ctx = make_runtime_context(frame_plan.frame_delta_seconds, output_frame_index);
                    m_callbacks.on_pre_update(ctx);
                }

                FrameStepperContext stepper_ctx{
                    .world = m_world,
                    .physics_system = m_physics_system,
                    .fixed_tick_serial = m_fixed_tick_serial,
                    .frame_serial = m_frame_serial,
                };
                m_frame_stepper.execute(frame_plan, stepper_ctx);

                if (m_callbacks.on_post_update) {
                    auto ctx = make_runtime_context(frame_plan.frame_delta_seconds, output_frame_index);
                    m_callbacks.on_post_update(ctx);
                }

                m_renderer.pipeline()->prepare_frame(system::render::FramePrepareContext{
                    .world = m_world,
                    .resources = m_resources,
                    .input = m_input_system,
                    .frame_serial = m_frame_serial,
                    .delta_seconds = frame_plan.frame_delta_seconds,
                });

                if (m_callbacks.on_pre_render) {
                    auto ctx = make_runtime_context(frame_plan.frame_delta_seconds, output_frame_index);
                    m_callbacks.on_pre_render(ctx);
                }

                m_renderer.draw_frame();

                if (m_callbacks.on_post_render) {
                    auto ctx = make_runtime_context(frame_plan.frame_delta_seconds, output_frame_index);
                    m_callbacks.on_post_render(ctx);
                }

                // Keep the non-interactive preview window responsive and visibly updating
                // while preserving the "no input system ownership" rule for offline mode.
                m_renderer.window().poll_events();

                m_resources.tick(m_frame_serial);
                ++m_frame_serial;
                ++result.frames_rendered;
            }
        } catch (const std::exception& e) {
            result.ok = false;
            result.error_message = e.what();
            log->error("OfflineRuntime main loop failed: {}", e.what());
        }

        result.fixed_ticks = m_fixed_tick_serial;

        try {
            if (m_callbacks.on_shutdown) {
                auto ctx = make_runtime_context(0.0, static_cast<std::uint32_t>(result.frames_rendered));
                m_callbacks.on_shutdown(ctx);
            }
        } catch (const std::exception& e) {
            log->error("OfflineRuntime shutdown callback failed: {}", e.what());
            if (result.ok) {
                result.ok = false;
                result.error_message = e.what();
            }
        }

        try {
            m_renderer.device().wait_idle();
            m_resources.flush_after_wait_idle();
        } catch (const std::exception& e) {
            log->error("OfflineRuntime wait_idle/flush_after_wait_idle failed: {}", e.what());
            if (result.ok) {
                result.ok = false;
                result.error_message = e.what();
            }
        }

        log->info(
            "OfflineRuntime finished (ok={}, frames_rendered={}, fixed_ticks={})",
            result.ok,
            result.frames_rendered,
            result.fixed_ticks
        );
        return result;
    }

private:
    OfflineRuntimeContext make_runtime_context(double frame_delta_seconds, std::uint32_t output_frame_index) {
        return OfflineRuntimeContext{
            .world = m_world,
            .resources = m_resources,
            .renderer = m_renderer,
            .physics_system = m_physics_system,
            .frame_serial = m_frame_serial,
            .output_frame_index = output_frame_index,
            .delta_seconds = frame_delta_seconds,
            .request_stop = [this]() { request_stop(); },
        };
    }
};

}  // namespace rtr::app
