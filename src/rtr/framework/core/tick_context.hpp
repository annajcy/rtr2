#pragma once

#include <cstdint>

namespace rtr::framework::core {

struct FixedTickContext {
    double fixed_delta_seconds{0.0};
    std::uint64_t fixed_tick_index{0};
};

struct FrameTickContext {
    double delta_seconds{0.0};
    double unscaled_delta_seconds{0.0};
    std::uint64_t frame_index{0};
};

} // namespace rtr::framework::core
