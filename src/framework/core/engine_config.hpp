#pragma once

#include <cstdint>
#include <string>

namespace rtr::framework::core {

struct EngineConfig {
    std::uint32_t window_width{800};
    std::uint32_t window_height{600};
    std::string window_title{"RTR2 Framework"};
    std::uint32_t max_frames_in_flight{2};

    double fixed_delta_seconds{1.0 / 60.0};
    std::uint32_t max_fixed_steps_per_frame{4};
    bool start_paused{false};
};

} // namespace rtr::framework::core
