#pragma once

#include <pbpt/math/math.h>

#include <cmath>
#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/physics/cloth/cloth_component.hpp"

namespace rtr::examples {

class BunnyClothSmokeDriver final : public framework::component::Component {
private:
    std::vector<pbpt::math::Vec3> m_rest_positions;
    double m_accumulated_time{0.0};

public:
    using Component::Component;

    float vertical_amplitude{0.05f};
    float lateral_amplitude{0.015f};
    float spatial_frequency{3.0f};
    float time_speed{2.0f};

    void on_fixed_update(const framework::core::FixedTickContext& ctx) override {
        if (!enabled()) {
            return;
        }

        auto* cloth = owner().get_component<framework::component::ClothComponent>();
        if (cloth == nullptr || !cloth->has_cloth()) {
            return;
        }

        auto& instance = cloth->cloth();
        if (m_rest_positions.empty()) {
            m_rest_positions = instance.topology.rest_positions;
            if (m_rest_positions.empty()) {
                return;
            }
        }

        m_accumulated_time += ctx.fixed_delta_seconds;
        for (std::size_t i = 0; i < m_rest_positions.size(); ++i) {
            const auto vertex_id = static_cast<system::physics::VertexID>(i);
            if (instance.state.pinned(vertex_id)) {
                instance.state.position(vertex_id) = m_rest_positions[i];
                instance.state.velocity(vertex_id) = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
                continue;
            }

            const auto& rest = m_rest_positions[i];
            const float wave_phase = rest.x() * spatial_frequency + rest.z() * (spatial_frequency * 0.7f) +
                                     static_cast<float>(m_accumulated_time * time_speed);
            const float lift = std::sin(wave_phase) * vertical_amplitude;
            const float sway = std::cos(wave_phase * 0.85f) * lateral_amplitude;
            const auto displaced = rest + pbpt::math::Vec3{sway, lift, 0.0f};
            instance.state.velocity(vertex_id) =
                (displaced - instance.state.position(vertex_id)) / static_cast<float>(ctx.fixed_delta_seconds);
            instance.state.position(vertex_id) = displaced;
        }
    }
};

}  // namespace rtr::examples
