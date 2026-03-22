#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace rtr::system::physics::ipc {

struct TetSurfaceResult {
    std::vector<uint32_t> surface_indices;    // triangle indices
    std::vector<uint32_t> surface_vertex_ids; // unique vertex IDs on surface
};

inline TetSurfaceResult extract_tet_surface(
    uint32_t vertex_count,
    std::span<const std::array<uint32_t, 4>> tets
) {
    // Face represented by sorted 3 vertex indices
    struct Face {
        uint32_t v[3];
        bool operator==(const Face& other) const {
            return v[0] == other.v[0] && v[1] == other.v[1] && v[2] == other.v[2];
        }
    };

    struct FaceHash {
        std::size_t operator()(const Face& f) const {
            std::size_t h = f.v[0];
            h ^= f.v[1] + 0x9e3779b9 + (h << 6) + (h >> 2);
            h ^= f.v[2] + 0x9e3779b9 + (h << 6) + (h >> 2);
            return h;
        }
    };

    // A face maps to the list of tet indices that share it. 
    // If it's a surface face, it will only have 1 tet.
    // However, to keep consistent winding order, we need the original unsorted vertices.
    // Instead of unsorted vertices in the key, we store a count, and the last seen oriented face.

    struct FaceData {
        int count = 0;
        std::array<uint32_t, 3> oriented_vertices;
    };

    std::unordered_map<Face, FaceData, FaceHash> face_map;

    auto add_face = [&](uint32_t a, uint32_t b, uint32_t c) {
        std::array<uint32_t, 3> sorted = {a, b, c};
        std::sort(sorted.begin(), sorted.end());
        Face f = {sorted[0], sorted[1], sorted[2]};

        auto& data = face_map[f];
        data.count++;
        // Maintain the outward orientation from the tet's perspective.
        data.oriented_vertices = {a, b, c}; 
    };

    for (const auto& tet : tets) {
        // Standard tet faces and outward winding assuming tet layout: (0,1,2) is base, 3 is top
        // Correct consistent winding depends on actual generator, usually:
        // (0,1,3), (1,2,3), (2,0,3), (0,2,1)
        add_face(tet[0], tet[1], tet[3]);
        add_face(tet[1], tet[2], tet[3]);
        add_face(tet[2], tet[0], tet[3]);
        add_face(tet[0], tet[2], tet[1]);
    }

    TetSurfaceResult result;
    std::vector<bool> is_surface_vertex(vertex_count, false);

    for (const auto& [_, data] : face_map) {
        if (data.count == 1) { // Surface face
            for (int i = 0; i < 3; ++i) {
                uint32_t v = data.oriented_vertices[i];
                result.surface_indices.push_back(v);
                if (!is_surface_vertex[v]) {
                    is_surface_vertex[v] = true;
                    result.surface_vertex_ids.push_back(v);
                }
            }
        }
    }

    std::sort(result.surface_vertex_ids.begin(), result.surface_vertex_ids.end());
    return result;
}

} // namespace rtr::system::physics::ipc
