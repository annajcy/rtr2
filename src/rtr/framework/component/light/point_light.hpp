#pragma once

#include "rtr/framework/component/component.hpp"
#include <pbpt/math/math.h>
#include "rtr/utils/log.hpp"
#include <stdexcept>
#include <cmath>

namespace rtr::framework::component::light {

class PointLight final : public Component {
public:
    pbpt::math::vec3 color{1.0f, 1.0f, 1.0f};
    float            intensity{10.0f};
    float            range{10.0f};
    float            specular_strength{1.0f};
    float            shininess{32.0f};

    explicit PointLight(core::GameObject& owner)
        : Component(owner) {}

    void set_color(const pbpt::math::vec3& c) { color = c; }

    void set_intensity(float i) {
        if (i < 0.0f || !std::isfinite(i)) {
            throw std::invalid_argument("PointLight intensity must be a non-negative finite value.");
        }
        intensity = i;
    }

    void set_range(float r) {
        if (r <= 0.0f || !std::isfinite(r)) {
            throw std::invalid_argument("PointLight range must be a positive finite value.");
        }
        range = r;
    }

    void set_specular_strength(float s) {
        if (s < 0.0f || !std::isfinite(s)) {
            throw std::invalid_argument("PointLight specular_strength must be a non-negative finite value.");
        }
        specular_strength = s;
    }

    void set_shininess(float s) {
        if (s < 1.0f || !std::isfinite(s)) {
            throw std::invalid_argument("PointLight shininess must be >= 1.0 and finite.");
        }
        shininess = s;
    }
};

}  // namespace rtr::framework::component::light
