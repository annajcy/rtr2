#pragma once

#include <pbpt/math/math.h>
#include <string>

namespace rtr::framework::integration {

struct PbptIntegratorRecord {
    std::string type{"path"};
    int         max_depth{-1};
};

struct PbptSensorRecord {
    pbpt::math::mat4 to_world{1.0f};
    float            fov_degrees{45.0f};
    float            near_clip{0.1f};
    float            far_clip{1000.0f};
    float            focus_distance{1000.0f};
    int              film_width{512};
    int              film_height{512};
    int              sample_count{4};
    std::string      fov_axis{"smaller"};
};

}  // namespace rtr::framework::integration
