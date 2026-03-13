#pragma once

#include <pbpt/math/math.h>
#include <vector>

#include "rtr/framework/component/component.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/rhi/device.hpp"
#include "rtr/system/physics/common/normal_recompute.hpp"

namespace rtr::examples {

class SineWaveDeformer final : public framework::component::Component {
private:
    std::vector<pbpt::math::Vec3> m_original_positions;
    std::vector<uint32_t> m_indices;
    double m_accumulated_time{0.0};

public:
    float spatial_frequency{5.0f};
    float time_speed{3.0f};
    float amplitude{0.1f};

    using Component::Component;

    void on_update(const framework::core::FrameTickContext& ctx) override {
        if (!enabled()) {
            return;
        }

        auto* renderer = owner().get_component<framework::component::DeformableMeshComponent>();
        if (renderer == nullptr) {
            return;
        }

        if (m_original_positions.empty()) {
            m_original_positions = renderer->local_vertices();
            m_indices = renderer->indices();
            if (m_original_positions.empty()) {
                return;
            }
        }

        m_accumulated_time += ctx.delta_seconds;

        std::vector<pbpt::math::Vec3> current_positions(m_original_positions.size());
        for (size_t i = 0; i < m_original_positions.size(); ++i) {
            const auto& op = m_original_positions[i];
            float wave = std::sin(op.x() * spatial_frequency + m_accumulated_time * time_speed) * 
                         std::cos(op.z() * spatial_frequency + m_accumulated_time * (time_speed * 0.666f));
            current_positions[i] = op + pbpt::math::Vec3{0.0f, wave * amplitude, 0.0f};
        }

        auto normals = system::physics::recompute_vertex_normals(current_positions, m_indices);

        std::vector<rhi::DynamicMesh::Vertex> update_vertices(current_positions.size());
        for (size_t i = 0; i < current_positions.size(); ++i) {
            update_vertices[i].position = current_positions[i];
            update_vertices[i].normal   = normals[i];
            update_vertices[i].uv       = pbpt::math::Vec2{0.0f};
        }

        renderer->update_positions(update_vertices);
    }
};

} // namespace rtr::examples
