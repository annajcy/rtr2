#pragma once

#include <algorithm>
#include <chrono>
#include <cstdint>
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
private:
    EngineConfig m_config{};
    std::unique_ptr<World> m_world{};
    bool m_stop_requested{false};
    double m_fixed_accumulator{0.0};
    std::uint64_t m_fixed_tick_index{0};
    std::uint64_t m_frame_index{0};

public:
    explicit Engine(EngineConfig config = {})
        : m_config(std::move(config)),
          m_world(std::make_unique<World>()) {}

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

    void request_stop() {
        m_stop_requested = true;
    }

    void reset_stop_request() {
        m_stop_requested = false;
    }

    bool stop_requested() const {
        return m_stop_requested;
    }

    std::uint64_t fixed_tick_index() const {
        return m_fixed_tick_index;
    }

    std::uint64_t frame_index() const {
        return m_frame_index;
    }

    double fixed_accumulator() const {
        return m_fixed_accumulator;
    }

    void run_frame(double frame_delta_seconds) {
        constexpr double kFixedTickEpsilon = 1e-12;
        double frame_delta = std::max(0.0, frame_delta_seconds);
        if (m_config.max_frame_delta_seconds > 0.0) {
            frame_delta = std::min(frame_delta, m_config.max_frame_delta_seconds);
        }
        const double fixed_dt = m_config.fixed_delta_seconds;

        if (fixed_dt > 0.0) {
            m_fixed_accumulator += frame_delta;
            std::uint32_t fixed_steps = 0;
            while ((m_fixed_accumulator + kFixedTickEpsilon) >= fixed_dt &&
                   fixed_steps < m_config.max_fixed_steps_per_frame) {
                fixed_tick(FixedTickContext{
                    .fixed_delta_seconds = fixed_dt,
                    .fixed_tick_index = m_fixed_tick_index++,
                });
                m_fixed_accumulator -= fixed_dt;
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

    void run_for_frames(std::uint64_t frame_count, double frame_delta_seconds = -1.0) {
        const double delta = frame_delta_seconds >= 0.0 ? frame_delta_seconds : m_config.fixed_delta_seconds;
        for (std::uint64_t i = 0; i < frame_count; ++i) {
            if (m_stop_requested) {
                break;
            }
            run_frame(delta);
        }
    }

    void run() {
        using Clock = std::chrono::steady_clock;
        auto previous_time = Clock::now();
        while (!m_stop_requested) {
            const auto current_time = Clock::now();
            double frame_delta =
                std::chrono::duration<double>(current_time - previous_time).count();
            previous_time = current_time;
            run_frame(frame_delta);
        }
    }
};

} // namespace rtr::framework::core
