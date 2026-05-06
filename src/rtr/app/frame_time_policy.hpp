#pragma once

#include <algorithm>
#include <concepts>
#include <cstdint>
#include <functional>
#include <stdexcept>

namespace rtr::app {

struct FrameExecutionPlan {
    std::uint32_t fixed_steps_to_run{0};
    double fixed_dt{0.0};
    double frame_delta_seconds{0.0};
};

template <typename T>
concept FrameTimePolicyConcept = requires(T& policy, bool paused) {
    { policy.make_plan(paused) } -> std::same_as<FrameExecutionPlan>;
};

class RealtimeFrameTimePolicy {
private:
    std::function<double()> m_now_seconds{};
    double m_fixed_delta_seconds{0.0};
    std::uint32_t m_max_fixed_steps_per_frame{0};
    double m_max_frame_delta_seconds{0.0};
    double m_previous_time_seconds{0.0};
    double m_accumulator{0.0};
    bool m_initialized{false};

public:
    explicit RealtimeFrameTimePolicy(
        double fixed_delta_seconds,
        std::uint32_t max_fixed_steps_per_frame,
        double max_frame_delta_seconds,
        std::function<double()> now_seconds
    )
        : m_now_seconds(std::move(now_seconds)),
          m_fixed_delta_seconds(fixed_delta_seconds),
          m_max_fixed_steps_per_frame(max_fixed_steps_per_frame),
          m_max_frame_delta_seconds(max_frame_delta_seconds) {
        if (!m_now_seconds) {
            throw std::invalid_argument("RealtimeFrameTimePolicy requires a valid now provider.");
        }
    }

    void reset() {
        m_previous_time_seconds = m_now_seconds();
        m_accumulator = 0.0;
        m_initialized = true;
    }

    FrameExecutionPlan make_plan(bool paused) {
        if (!m_initialized) {
            reset();
        }

        const double current_time_seconds = m_now_seconds();
        double frame_delta = current_time_seconds - m_previous_time_seconds;
        m_previous_time_seconds = current_time_seconds;

        if (frame_delta < 0.0) {
            frame_delta = 0.0;
        }
        if (m_max_frame_delta_seconds > 0.0) {
            frame_delta = std::min(frame_delta, m_max_frame_delta_seconds);
        }

        if (paused) {
            return {};
        }

        FrameExecutionPlan plan{
            .fixed_steps_to_run = 0,
            .fixed_dt = m_fixed_delta_seconds,
            .frame_delta_seconds = frame_delta,
        };

        if (m_fixed_delta_seconds <= 0.0 || m_max_fixed_steps_per_frame == 0) {
            return plan;
        }

        m_accumulator += frame_delta;
        while (m_accumulator >= m_fixed_delta_seconds &&
               plan.fixed_steps_to_run < m_max_fixed_steps_per_frame) {
            m_accumulator -= m_fixed_delta_seconds;
            ++plan.fixed_steps_to_run;
        }

        return plan;
    }
};

class OfflineFrameTimePolicy {
private:
    double m_sim_dt{0.0};
    std::uint32_t m_steps_per_output_frame{1};

public:
    explicit OfflineFrameTimePolicy(
        double sim_dt,
        std::uint32_t steps_per_output_frame
    )
        : m_sim_dt(sim_dt),
          m_steps_per_output_frame(steps_per_output_frame) {}

    FrameExecutionPlan make_plan(bool paused) const {
        if (paused) {
            return {};
        }

        return FrameExecutionPlan{
            .fixed_steps_to_run = m_steps_per_output_frame,
            .fixed_dt = m_sim_dt,
            .frame_delta_seconds = m_sim_dt * static_cast<double>(m_steps_per_output_frame),
        };
    }
};

} // namespace rtr::app
