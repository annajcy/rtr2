#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "framework/core/tick_context.hpp"
#include "framework/core/world.hpp"

namespace rtr::framework::core {

struct EngineConfig {
    std::uint32_t window_width{800};
    std::uint32_t window_height{600};
    std::string window_title{"RTR2 Framework"};
    std::uint32_t max_frames_in_flight{2};

    double fixed_delta_seconds{1.0 / 60.0};
    std::uint32_t max_fixed_steps_per_frame{4};
    double max_frame_delta_seconds{0.1};
    bool start_paused{false};
};

class Engine {
public:
    struct LoopHooks {
        std::function<void()> input_begin{};
        std::function<void()> input_poll{};
        std::function<void()> input_end{};
        std::function<void()> render{};
        std::function<bool()> should_close{};
        std::function<double()> now_seconds{};
    };

private:
    EngineConfig m_config{};
    std::unique_ptr<World> m_world{};
    LoopHooks m_hooks{};
    bool m_stop_requested{false};
    bool m_paused{false};
    std::uint64_t m_fixed_tick_index{0};
    std::uint64_t m_frame_index{0};

public:
    explicit Engine(EngineConfig config = {})
        : m_config(std::move(config)),
          m_world(std::make_unique<World>()),
          m_paused(m_config.start_paused) {}

    const EngineConfig& config() const {
        return m_config;
    }

    World& world() {
        return *m_world;
    }

    const World& world() const {
        return *m_world;
    }

    void fixed_tick(const FixedTickContext& ctx) {
        m_world->fixed_tick(ctx);
    }

    void tick(const FrameTickContext& ctx) {
        m_world->tick(ctx);
    }

    void late_tick(const FrameTickContext& ctx) {
        m_world->late_tick(ctx);
    }

    void set_loop_hooks(LoopHooks hooks) {
        m_hooks = std::move(hooks);
    }

    const LoopHooks& loop_hooks() const {
        return m_hooks;
    }

    void request_stop() {
        m_stop_requested = true;
    }

    void reset_stop_request() {
        m_stop_requested = false;
    }

    bool stop_requested() const {
        return m_stop_requested;
    }

    void set_paused(bool paused) {
        m_paused = paused;
    }

    bool paused() const {
        return m_paused;
    }

    std::uint64_t fixed_tick_index() const {
        return m_fixed_tick_index;
    }

    std::uint64_t frame_index() const {
        return m_frame_index;
    }

    void run() {
        const auto default_now = []() -> double {
            using Clock = std::chrono::steady_clock;
            const auto now = Clock::now().time_since_epoch();
            return std::chrono::duration<double>(now).count();
        };
        const auto now_fn = m_hooks.now_seconds ? m_hooks.now_seconds : default_now;
        double previous_time = now_fn();
        double accumulator = 0.0;

        while (!m_stop_requested) {
            if (m_hooks.should_close && m_hooks.should_close()) {
                break;
            }

            if (m_hooks.input_begin) {
                m_hooks.input_begin();
            }
            if (m_hooks.input_poll) {
                m_hooks.input_poll();
            }

            const double current_time = now_fn();
            double frame_delta = current_time - previous_time;
            previous_time = current_time;
            if (frame_delta < 0.0) {
                frame_delta = 0.0;
            }
            if (m_config.max_frame_delta_seconds > 0.0) {
                frame_delta = std::min(frame_delta, m_config.max_frame_delta_seconds);
            }

            if (!m_paused) {
                const double fixed_dt = m_config.fixed_delta_seconds;
                if (fixed_dt > 0.0) {
                    accumulator += frame_delta;
                    std::uint32_t fixed_steps = 0;
                    while (accumulator >= fixed_dt && fixed_steps < m_config.max_fixed_steps_per_frame) {
                        fixed_tick(FixedTickContext{
                            .fixed_delta_seconds = fixed_dt,
                            .fixed_tick_index = m_fixed_tick_index++,
                        });
                        accumulator -= fixed_dt;
                        ++fixed_steps;
                    }
                }

                FrameTickContext frame_ctx{
                    .delta_seconds = frame_delta,
                    .unscaled_delta_seconds = frame_delta,
                    .frame_index = m_frame_index++,
                };
                tick(frame_ctx);
                late_tick(frame_ctx);
            }

            if (m_hooks.render) {
                m_hooks.render();
            }

            if (m_hooks.input_end) {
                m_hooks.input_end();
            }
        }
    }
};

} // namespace rtr::framework::core
