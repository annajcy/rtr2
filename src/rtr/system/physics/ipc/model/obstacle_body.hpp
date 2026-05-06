#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <vector>

#include <Eigen/Core>

#include "rtr/system/physics/ipc/model/ipc_body.hpp"

namespace rtr::system::physics::ipc {

struct ObstacleBody {
    IPCBodyInfo info{.type = IPCBodyType::Obstacle};
    std::vector<Eigen::Vector3d> positions{};
    std::vector<std::array<std::size_t, 3>> triangles{};
    std::vector<std::array<std::size_t, 2>> edges{};

    std::size_t vertex_count() const { return positions.size(); }
    std::size_t triangle_count() const { return triangles.size(); }
    std::size_t edge_count() const { return edges.size(); }

    void validate() const {
        for (const auto& triangle : triangles) {
            for (const std::size_t vertex_index : triangle) {
                if (vertex_index >= positions.size()) {
                    throw std::out_of_range("ObstacleBody triangle vertex index out of range.");
                }
            }
        }
    }

    void build_edges() {
        validate();

        std::vector<std::array<std::size_t, 2>> unique_edges{};
        unique_edges.reserve(triangles.size() * 3u);

        auto append_edge = [&unique_edges](std::size_t a, std::size_t b) {
            if (a == b) {
                throw std::invalid_argument("ObstacleBody edge endpoints must be distinct.");
            }
            if (a > b) {
                std::swap(a, b);
            }
            unique_edges.push_back({a, b});
        };

        for (const auto& triangle : triangles) {
            append_edge(triangle[0], triangle[1]);
            append_edge(triangle[1], triangle[2]);
            append_edge(triangle[0], triangle[2]);
        }

        std::sort(unique_edges.begin(), unique_edges.end());
        unique_edges.erase(std::unique(unique_edges.begin(), unique_edges.end()), unique_edges.end());
        edges = std::move(unique_edges);
        info.vertex_count = vertex_count();
    }
};

}  // namespace rtr::system::physics::ipc
