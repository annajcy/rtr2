#pragma once

#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <tiny_obj_loader.h>

#include <glm/geometric.hpp>
#include <glm/glm.hpp>

#include "rtr/utils/obj_types.hpp"

namespace rtr::utils {

namespace obj_io_detail {

struct VertexKey {
    int pos{0};
    int tex{0};
    int norm{0};

    bool operator==(const VertexKey& other) const {
        return pos == other.pos && tex == other.tex && norm == other.norm;
    }
};

struct VertexKeyHash {
    size_t operator()(const VertexKey& key) const noexcept {
        const size_t h1 = std::hash<int>{}(key.pos);
        const size_t h2 = std::hash<int>{}(key.tex);
        const size_t h3 = std::hash<int>{}(key.norm);
        return h1 ^ (h2 << 1u) ^ (h3 << 2u);
    }
};

inline glm::vec3 read_position(
    const std::vector<tinyobj::real_t>& vertices,
    int index,
    const std::string& filepath
) {
    const size_t base = static_cast<size_t>(index) * 3u;
    if (base + 2u >= vertices.size()) {
        throw std::runtime_error("OBJ vertex index out of range in " + filepath);
    }
    return glm::vec3(
        static_cast<float>(vertices[base + 0u]),
        static_cast<float>(vertices[base + 1u]),
        static_cast<float>(vertices[base + 2u])
    );
}

inline glm::vec2 read_texcoord(
    const std::vector<tinyobj::real_t>& texcoords,
    int index,
    const std::string& filepath
) {
    const size_t base = static_cast<size_t>(index) * 2u;
    if (base + 1u >= texcoords.size()) {
        throw std::runtime_error("OBJ texcoord index out of range in " + filepath);
    }
    return glm::vec2(
        static_cast<float>(texcoords[base + 0u]),
        static_cast<float>(texcoords[base + 1u])
    );
}

inline glm::vec3 read_normal(
    const std::vector<tinyobj::real_t>& normals,
    int index,
    const std::string& filepath
) {
    const size_t base = static_cast<size_t>(index) * 3u;
    if (base + 2u >= normals.size()) {
        throw std::runtime_error("OBJ normal index out of range in " + filepath);
    }
    return glm::vec3(
        static_cast<float>(normals[base + 0u]),
        static_cast<float>(normals[base + 1u]),
        static_cast<float>(normals[base + 2u])
    );
}

inline void ensure_parent_directory(const std::filesystem::path& path) {
    if (!path.has_parent_path()) {
        return;
    }
    std::filesystem::create_directories(path.parent_path());
}

} // namespace obj_io_detail

inline ObjMeshData load_obj_from_path(const std::string& filepath) {
    if (filepath.empty()) {
        throw std::invalid_argument("OBJ path must not be empty.");
    }

    tinyobj::ObjReaderConfig config{};
    config.triangulate = true;

    tinyobj::ObjReader reader{};
    if (!reader.ParseFromFile(filepath, config)) {
        std::string err = reader.Error();
        if (err.empty()) {
            err = reader.Warning();
        }
        throw std::runtime_error(
            "Failed to parse OBJ file with tinyobjloader: " + filepath +
            (err.empty() ? "" : " | " + err)
        );
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();

    ObjMeshData data{};
    std::unordered_map<obj_io_detail::VertexKey, std::uint32_t, obj_io_detail::VertexKeyHash> vertex_lookup{};
    const bool has_input_normals = !attrib.normals.empty();

    for (const auto& shape : shapes) {
        for (const auto& idx : shape.mesh.indices) {
            if (idx.vertex_index < 0) {
                throw std::runtime_error("OBJ face references missing position data in " + filepath);
            }

            const obj_io_detail::VertexKey key{idx.vertex_index, idx.texcoord_index, idx.normal_index};

            const auto found = vertex_lookup.find(key);
            if (found != vertex_lookup.end()) {
                data.indices.push_back(found->second);
                continue;
            }

            ObjVertex vertex{};
            vertex.position = obj_io_detail::read_position(attrib.vertices, idx.vertex_index, filepath);
            if (idx.texcoord_index >= 0) {
                vertex.uv = obj_io_detail::read_texcoord(attrib.texcoords, idx.texcoord_index, filepath);
            }
            if (idx.normal_index >= 0) {
                vertex.normal = obj_io_detail::read_normal(attrib.normals, idx.normal_index, filepath);
            }

            const std::uint32_t new_index = static_cast<std::uint32_t>(data.vertices.size());
            data.vertices.push_back(vertex);
            vertex_lookup.emplace(key, new_index);
            data.indices.push_back(new_index);
        }
    }

    if (!has_input_normals) {
        std::vector<glm::vec3> accum_normals(data.vertices.size(), glm::vec3(0.0f));
        for (size_t i = 0; i + 2u < data.indices.size(); i += 3u) {
            const auto i0 = data.indices[i + 0u];
            const auto i1 = data.indices[i + 1u];
            const auto i2 = data.indices[i + 2u];
            const glm::vec3& p0 = data.vertices[i0].position;
            const glm::vec3& p1 = data.vertices[i1].position;
            const glm::vec3& p2 = data.vertices[i2].position;
            const glm::vec3 face_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
            accum_normals[i0] += face_normal;
            accum_normals[i1] += face_normal;
            accum_normals[i2] += face_normal;
        }
        for (size_t v = 0; v < data.vertices.size(); ++v) {
            if (glm::length(accum_normals[v]) > 0.0f) {
                data.vertices[v].normal = glm::normalize(accum_normals[v]);
            } else {
                data.vertices[v].normal = glm::vec3(0.0f, 1.0f, 0.0f);
            }
        }
    }

    return data;
}

inline void write_obj_to_path(const ObjMeshData& mesh, const std::string& path) {
    if (path.empty()) {
        throw std::invalid_argument("OBJ output path must not be empty.");
    }
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::invalid_argument("ObjMeshData must not be empty.");
    }
    if (mesh.indices.size() % 3u != 0u) {
        throw std::invalid_argument("ObjMeshData indices must be triangles (size % 3 == 0).");
    }

    const std::filesystem::path out_path(path);
    obj_io_detail::ensure_parent_directory(out_path);

    std::ofstream out(out_path, std::ios::binary);
    if (!out.is_open()) {
        throw std::runtime_error("Failed to open OBJ output path: " + path);
    }

    out << std::setprecision(9);
    for (const auto& vertex : mesh.vertices) {
        out << "v " << vertex.position.x << ' ' << vertex.position.y << ' ' << vertex.position.z << '\n';
    }
    for (const auto& vertex : mesh.vertices) {
        out << "vt " << vertex.uv.x << ' ' << vertex.uv.y << '\n';
    }
    for (const auto& vertex : mesh.vertices) {
        out << "vn " << vertex.normal.x << ' ' << vertex.normal.y << ' ' << vertex.normal.z << '\n';
    }

    const std::uint32_t max_index = static_cast<std::uint32_t>(mesh.vertices.size());
    for (size_t i = 0; i < mesh.indices.size(); i += 3u) {
        const std::uint32_t i0 = mesh.indices[i + 0u];
        const std::uint32_t i1 = mesh.indices[i + 1u];
        const std::uint32_t i2 = mesh.indices[i + 2u];
        if (i0 >= max_index || i1 >= max_index || i2 >= max_index) {
            throw std::runtime_error("ObjMeshData indices out of range when writing OBJ: " + path);
        }

        const std::uint32_t f0 = i0 + 1u;
        const std::uint32_t f1 = i1 + 1u;
        const std::uint32_t f2 = i2 + 1u;
        out << "f "
            << f0 << '/' << f0 << '/' << f0 << ' '
            << f1 << '/' << f1 << '/' << f1 << ' '
            << f2 << '/' << f2 << '/' << f2 << '\n';
    }

    if (!out.good()) {
        throw std::runtime_error("Failed to write OBJ file: " + path);
    }
}

} // namespace rtr::utils
