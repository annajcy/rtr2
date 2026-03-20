#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

#include "rtr/system/physics/cloth/cloth_spring.hpp"
#include "rtr/system/physics/cloth/cloth_state.hpp"

namespace rtr::system::physics {

struct ClothParams {
    pbpt::math::Float default_vertex_mass{1.0f};
    pbpt::math::Vec3 gravity{0.0f, -9.81f, 0.0f};
    pbpt::math::Float edge_stiffness{800.0f};
    pbpt::math::Float bend_stiffness{80.0f};
    pbpt::math::Float spring_damping{8.0f};
    pbpt::math::Float velocity_damping{0.02f};
    std::uint32_t substeps{8};
};

struct ClothInstance {
    ClothTopology topology;
    ClothState state;
    ClothParams params;
    ClothSpringNetwork spring_network{};
};

class ClothWorld {
private:
    std::unordered_map<ClothID, ClothInstance> m_instances{};
    ClothID m_next_cloth_id{0};

    static constexpr pbpt::math::Float kLengthEpsilon = 1e-6f;

    static bool is_finite_vec3(const pbpt::math::Vec3& value) {
        return std::isfinite(value.x()) && std::isfinite(value.y()) && std::isfinite(value.z());
    }

    static void validate_params(const ClothParams& params) {
        if (!std::isfinite(params.default_vertex_mass) || params.default_vertex_mass <= 0.0f) {
            throw std::invalid_argument("Cloth default_vertex_mass must be finite and positive.");
        }
        if (!is_finite_vec3(params.gravity)) {
            throw std::invalid_argument("Cloth gravity must be finite.");
        }
        if (!std::isfinite(params.edge_stiffness) || params.edge_stiffness < 0.0f) {
            throw std::invalid_argument("Cloth edge_stiffness must be finite and non-negative.");
        }
        if (!std::isfinite(params.bend_stiffness) || params.bend_stiffness < 0.0f) {
            throw std::invalid_argument("Cloth bend_stiffness must be finite and non-negative.");
        }
        if (!std::isfinite(params.spring_damping) || params.spring_damping < 0.0f) {
            throw std::invalid_argument("Cloth spring_damping must be finite and non-negative.");
        }
        if (!std::isfinite(params.velocity_damping) || params.velocity_damping < 0.0f) {
            throw std::invalid_argument("Cloth velocity_damping must be finite and non-negative.");
        }
        if (params.substeps == 0u) {
            throw std::invalid_argument("Cloth substeps must be at least 1.");
        }
    }

    static void validate_instance(const ClothInstance& instance) {
        validate_params(instance.params);

        const auto vertex_count = instance.topology.rest_positions.size();
        if (vertex_count == 0u) {
            throw std::invalid_argument("Cloth topology must contain at least one vertex.");
        }
        if (instance.state.positions.size() != vertex_count ||
            instance.state.velocities.size() != vertex_count ||
            instance.state.masses.size() != vertex_count ||
            instance.state.pinned_mask.size() != vertex_count) {
            throw std::invalid_argument("Cloth state arrays must match topology vertex count.");
        }
        if (instance.topology.edge_rest_lengths.size() != instance.topology.edges.size()) {
            throw std::invalid_argument("Cloth topology edge_rest_lengths must match edge count.");
        }

        for (std::size_t i = 0; i < vertex_count; ++i) {
            if (!is_finite_vec3(instance.state.positions[i]) || !is_finite_vec3(instance.state.velocities[i])) {
                throw std::invalid_argument("Cloth state positions and velocities must be finite.");
            }
            const auto mass = instance.state.masses[i];
            if (!std::isfinite(mass) || mass <= 0.0f) {
                throw std::invalid_argument("Cloth vertex masses must be finite and positive.");
            }
        }
    }

    static void enforce_pinned_vertices(ClothInstance& instance) {
        for (std::size_t vertex_index = 0; vertex_index < instance.state.vertex_count(); ++vertex_index) {
            const auto vertex_id = static_cast<VertexID>(vertex_index);
            if (!instance.state.pinned(vertex_id)) {
                continue;
            }
            instance.state.positions[vertex_index] = instance.topology.rest_positions[vertex_index];
            instance.state.velocities[vertex_index] = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
        }
    }

    static pbpt::math::Float stiffness_for(const ClothParams& params, ClothSpringKind kind) {
        return kind == ClothSpringKind::Edge ? params.edge_stiffness : params.bend_stiffness;
    }

    static void step_instance(ClothInstance& instance, float delta_seconds) {
        if (delta_seconds <= 0.0f) {
            return;
        }

        const auto vertex_count = instance.state.vertex_count();
        if (vertex_count == 0u) {
            return;
        }

        const auto substep_dt = delta_seconds / static_cast<float>(instance.params.substeps);
        if (substep_dt <= 0.0f) {
            return;
        }

        std::vector<pbpt::math::Vec3> forces(vertex_count, pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

        for (std::uint32_t substep_index = 0; substep_index < instance.params.substeps; ++substep_index) {
            std::fill(forces.begin(), forces.end(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

            for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
                const auto vertex_id = static_cast<VertexID>(vertex_index);
                if (instance.state.pinned(vertex_id)) {
                    continue;
                }
                forces[vertex_index] += instance.params.gravity * instance.state.masses[vertex_index];
            }

            for (const auto& spring : instance.spring_network.springs) {
                const auto vertex_index_a = static_cast<std::size_t>(spring.vertices[0]);
                const auto vertex_index_b = static_cast<std::size_t>(spring.vertices[1]);
                const auto delta = instance.state.positions[vertex_index_b] - instance.state.positions[vertex_index_a];
                const auto current_length = pbpt::math::length(delta);
                if (!std::isfinite(current_length) || current_length <= kLengthEpsilon) {
                    continue;
                }

                const auto direction = delta / current_length;
                const auto extension = current_length - spring.rest_length;
                const auto relative_speed = pbpt::math::dot(
                    instance.state.velocities[vertex_index_b] - instance.state.velocities[vertex_index_a],
                    direction
                );
                const auto total_force =
                    (stiffness_for(instance.params, spring.kind) * extension +
                     instance.params.spring_damping * relative_speed) *
                    direction;

                forces[vertex_index_a] += total_force;
                forces[vertex_index_b] -= total_force;
            }

            const auto damping_factor =
                std::max(0.0f, 1.0f - instance.params.velocity_damping * static_cast<pbpt::math::Float>(substep_dt));

            for (std::size_t vertex_index = 0; vertex_index < vertex_count; ++vertex_index) {
                const auto vertex_id = static_cast<VertexID>(vertex_index);
                if (instance.state.pinned(vertex_id)) {
                    continue;
                }

                const auto mass = instance.state.masses[vertex_index];
                instance.state.velocities[vertex_index] += (forces[vertex_index] / mass) * substep_dt;
                instance.state.velocities[vertex_index] *= damping_factor;
                instance.state.positions[vertex_index] += instance.state.velocities[vertex_index] * substep_dt;
            }

            enforce_pinned_vertices(instance);
        }
    }

public:
    ClothID create_cloth(ClothInstance instance) {
        validate_instance(instance);
        instance.spring_network = build_cloth_spring_network(instance.topology);

        const ClothID id = m_next_cloth_id++;
        m_instances.emplace(id, std::move(instance));
        return id;
    }

    bool remove_cloth(ClothID id) {
        return m_instances.erase(id) > 0u;
    }

    bool has_cloth(ClothID id) const {
        return m_instances.find(id) != m_instances.end();
    }

    ClothInstance& get_cloth(ClothID id) {
        const auto it = m_instances.find(id);
        if (it == m_instances.end()) {
            throw std::out_of_range("ClothID is invalid.");
        }
        return it->second;
    }

    const ClothInstance& get_cloth(ClothID id) const {
        const auto it = m_instances.find(id);
        if (it == m_instances.end()) {
            throw std::out_of_range("ClothID is invalid.");
        }
        return it->second;
    }

    std::size_t cloth_count() const {
        return m_instances.size();
    }

    void step(float delta_seconds) {
        if (!std::isfinite(delta_seconds) || delta_seconds < 0.0f) {
            throw std::invalid_argument("ClothWorld step delta_seconds must be finite and non-negative.");
        }
        if (delta_seconds == 0.0f) {
            return;
        }

        for (auto& [_, instance] : m_instances) {
            step_instance(instance, delta_seconds);
        }
    }
};

}  // namespace rtr::system::physics
