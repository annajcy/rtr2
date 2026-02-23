#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rtr/framework/core/tick_context.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/rhi/frame_constants.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/pipeline.hpp"
#include "rtr/system/render/renderer.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::app {

struct AppRuntimeConfig {
    std::uint32_t window_width{800};
    std::uint32_t window_height{600};
    std::string   window_title{"RTR2 AppRuntime"};
    std::string   resource_root_dir{"./assets/"};

    double           fixed_delta_seconds{1.0 / 60.0};
    std::uint32_t    max_fixed_steps_per_frame{4};
    double           max_frame_delta_seconds{0.1};
    bool             start_paused{false};
    bool             auto_init_logging{true};
    utils::LogConfig log_config{};
};

struct RuntimeResult {
    bool          ok{true};
    std::string   error_message{};
    std::uint64_t frames_rendered{0};
    std::uint64_t fixed_ticks{0};
};

struct RuntimeContext {
    framework::core::World&          world;
    resource::ResourceManager&       resources;
    system::render::Renderer&        renderer;
    system::input::InputSystem&      input;
    std::uint64_t                    frame_serial{0};
    double                           delta_seconds{0.0};
    std::function<void()>            request_stop{};
    bool                             paused{false};
};

struct RuntimeCallbacks {
    std::function<void(RuntimeContext&)> on_startup{};
    std::function<void(RuntimeContext&)> on_input{};
    std::function<void(RuntimeContext&)> on_pre_update{};
    std::function<void(RuntimeContext&)> on_post_update{};
    std::function<void(RuntimeContext&)> on_pre_render{};
    std::function<void(RuntimeContext&)> on_post_render{};
    std::function<void(RuntimeContext&)> on_shutdown{};
};

class AppRuntime {
private:
    static AppRuntimeConfig prepare_config(AppRuntimeConfig config) {
        if (config.auto_init_logging) {
            utils::init_logging(config.log_config);
        }
        return config;
    }

    static std::shared_ptr<spdlog::logger> logger() { return utils::get_logger("app.runtime"); }

    AppRuntimeConfig m_config{};
    RuntimeCallbacks m_callbacks{};

    resource::ResourceManager        m_resources;
    framework::core::World           m_world;
    system::render::Renderer         m_renderer;
    system::input::InputSystem       m_input;

    bool          m_stop_requested{false};
    bool          m_paused{false};
    std::uint64_t m_frame_serial{0};
    std::uint64_t m_fixed_tick_index{0};

public:
    explicit AppRuntime(AppRuntimeConfig config = {})
        : m_config(prepare_config(std::move(config))),
          m_resources(m_config.resource_root_dir),
          m_world(m_resources),
          m_renderer(static_cast<int>(m_config.window_width), static_cast<int>(m_config.window_height),
                     m_config.window_title),
          m_input(&m_renderer.window()),
          m_paused(m_config.start_paused) {
        logger()->info("AppRuntime initialized (window={}x{}, title='{}', frames_in_flight={}, paused={})",
                       m_config.window_width, m_config.window_height, m_config.window_title,
                       rhi::kFramesInFlight, m_paused);
    }

    const AppRuntimeConfig& config() const { return m_config; }

    framework::core::World& world() { return m_world; }
    const framework::core::World& world() const { return m_world; }

    resource::ResourceManager& resource_manager() { return m_resources; }
    const resource::ResourceManager& resource_manager() const { return m_resources; }

    system::render::Renderer& renderer() { return m_renderer; }
    const system::render::Renderer& renderer() const { return m_renderer; }

    system::input::InputSystem& input_system() { return m_input; }
    const system::input::InputSystem& input_system() const { return m_input; }

    system::render::IRenderPipeline* pipeline() { return m_renderer.pipeline(); }
    const system::render::IRenderPipeline* pipeline() const { return m_renderer.pipeline(); }

    void set_callbacks(RuntimeCallbacks callbacks) { m_callbacks = std::move(callbacks); }
    const RuntimeCallbacks& callbacks() const { return m_callbacks; }

    void set_pipeline(std::unique_ptr<system::render::IRenderPipeline> pipeline) {
        auto log = logger();
        if (!pipeline) {
            log->error("set_pipeline received null pipeline.");
            throw std::runtime_error("AppRuntime set_pipeline received null pipeline.");
        }

        m_renderer.set_pipeline(std::move(pipeline));
        log->info("Pipeline bound to runtime.");
    }

    void request_stop() { m_stop_requested = true; }
    bool stop_requested() const { return m_stop_requested; }

    void set_paused(bool paused) { m_paused = paused; }
    bool paused() const { return m_paused; }

    RuntimeResult run() {
        auto          log = logger();
        RuntimeResult result{};
        if (m_renderer.pipeline() == nullptr) {
            result.ok            = false;
            result.error_message = "AppRuntime requires pipeline before run().";
            log->error("run() aborted: pipeline is not bound.");
            return result;
        }
        log->info("Run started (paused={}, fixed_dt={}, max_fixed_steps_per_frame={}, max_frame_delta={})", m_paused,
                  m_config.fixed_delta_seconds, m_config.max_fixed_steps_per_frame, m_config.max_frame_delta_seconds);

        const auto default_now = []() -> double {
            using Clock    = std::chrono::steady_clock;
            const auto now = Clock::now().time_since_epoch();
            return std::chrono::duration<double>(now).count();
        };

        try {
            double previous_time = default_now();
            double accumulator   = 0.0;

            if (m_callbacks.on_startup) {
                auto ctx = make_runtime_context(0.0);
                m_callbacks.on_startup(ctx);
            }

            while (!m_stop_requested && !m_renderer.window().is_should_close()) {
                m_input.begin_frame();
                m_renderer.window().poll_events();

                if (m_callbacks.on_input) {
                    auto ctx = make_runtime_context(0.0);
                    m_callbacks.on_input(ctx);
                }

                const double current_time = default_now();
                double       frame_delta  = current_time - previous_time;
                previous_time             = current_time;

                if (frame_delta < 0.0) {
                    frame_delta = 0.0;
                }
                if (m_config.max_frame_delta_seconds > 0.0) {
                    frame_delta = std::min(frame_delta, m_config.max_frame_delta_seconds);
                }

                if (m_callbacks.on_pre_update) {
                    auto ctx = make_runtime_context(frame_delta);
                    m_callbacks.on_pre_update(ctx);
                }

                if (!m_paused) {
                    if (m_config.fixed_delta_seconds > 0.0) {
                        accumulator += frame_delta;
                        std::uint32_t fixed_steps = 0;
                        while (accumulator >= m_config.fixed_delta_seconds &&
                               fixed_steps < m_config.max_fixed_steps_per_frame) {
                            m_world.fixed_tick(framework::core::FixedTickContext{
                                .fixed_delta_seconds = m_config.fixed_delta_seconds,
                                .fixed_tick_index    = m_fixed_tick_index++,
                            });
                            accumulator -= m_config.fixed_delta_seconds;
                            ++fixed_steps;
                        }
                    }

                    framework::core::FrameTickContext tick_ctx{
                        .delta_seconds          = frame_delta,
                        .unscaled_delta_seconds = frame_delta,
                        .frame_index            = m_frame_serial,
                    };
                    m_world.tick(tick_ctx);
                    m_world.late_tick(tick_ctx);
                }

                if (m_callbacks.on_post_update) {
                    auto ctx = make_runtime_context(frame_delta);
                    m_callbacks.on_post_update(ctx);
                }

                if (auto* frame_prepare = dynamic_cast<system::render::IFramePreparePipeline*>(m_renderer.pipeline());
                    frame_prepare != nullptr) {
                    frame_prepare->prepare_frame(system::render::FramePrepareContext{
                        .world         = m_world,
                        .resources     = m_resources,
                        .input         = m_input,
                        .frame_serial  = m_frame_serial,
                        .delta_seconds = frame_delta,
                    });
                }

                if (m_callbacks.on_pre_render) {
                    auto ctx = make_runtime_context(frame_delta);
                    m_callbacks.on_pre_render(ctx);
                }

                m_renderer.draw_frame();

                if (m_callbacks.on_post_render) {
                    auto ctx = make_runtime_context(frame_delta);
                    m_callbacks.on_post_render(ctx);
                }

                // Keep per-frame mouse deltas available through update/render,
                // then clear them at frame tail.
                m_input.end_frame();

                m_resources.tick(m_frame_serial);
                ++m_frame_serial;
                ++result.frames_rendered;
            }
        } catch (const std::exception& e) {
            result.ok            = false;
            result.error_message = e.what();
            log->error("Runtime main loop failed: {}", e.what());
        }

        result.fixed_ticks = m_fixed_tick_index;

        try {
            if (m_callbacks.on_shutdown) {
                auto ctx = make_runtime_context(0.0);
                m_callbacks.on_shutdown(ctx);
            }
        } catch (const std::exception& e) {
            log->error("Shutdown callback failed: {}", e.what());
            if (result.ok) {
                result.ok            = false;
                result.error_message = e.what();
            }
        }

        try {
            m_renderer.device().wait_idle();
            m_resources.flush_after_wait_idle();
        } catch (const std::exception& e) {
            log->error("wait_idle/flush_after_wait_idle failed: {}", e.what());
            if (result.ok) {
                result.ok            = false;
                result.error_message = e.what();
            }
        }

        log->info("Run finished (ok={}, frames_rendered={}, fixed_ticks={})", result.ok, result.frames_rendered,
                  result.fixed_ticks);
        return result;
    }

private:
    RuntimeContext make_runtime_context(double delta_seconds) {
        return RuntimeContext{
            .world         = m_world,
            .resources     = m_resources,
            .renderer      = m_renderer,
            .input         = m_input,
            .frame_serial  = m_frame_serial,
            .delta_seconds = delta_seconds,
            .request_stop  = [this]() { request_stop(); },
            .paused        = m_paused,
        };
    }
};

}  // namespace rtr::app
