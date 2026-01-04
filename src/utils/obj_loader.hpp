#pragma once

#include <cstdint>
#include <functional>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include <glm/glm.hpp>
#include <tiny_obj_loader.h>

namespace rtr::utils {

struct ObjVertex {
    glm::vec3 position{0.0f};
    glm::vec2 uv{0.0f};
    glm::vec3 normal{0.0f};
};

struct ObjMeshData {
    std::vector<ObjVertex> vertices;
    std::vector<uint32_t> indices;
};

namespace detail {
struct VertexKey {
    int pos{0};
    int tex{0};
    int norm{0};

    bool operator==(const VertexKey& other) const {
        return pos == other.pos && tex == other.tex && norm == other.norm;
    }
};

struct VertexKeyHash {
    size_t operator()(const VertexKey& k) const noexcept {
        size_t h1 = std::hash<int>{}(k.pos);
        size_t h2 = std::hash<int>{}(k.tex);
        size_t h3 = std::hash<int>{}(k.norm);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

inline glm::vec3 read_position(const std::vector<tinyobj::real_t>& vertices,
                               int index,
                               const std::string& filepath) {
    size_t base = static_cast<size_t>(index) * 3;
    if (base + 2 >= vertices.size()) {
        throw std::runtime_error("OBJ vertex index out of range in " + filepath);
    }
    return glm::vec3(
        static_cast<float>(vertices[base]),
        static_cast<float>(vertices[base + 1]),
        static_cast<float>(vertices[base + 2]));
}

inline glm::vec2 read_texcoord(const std::vector<tinyobj::real_t>& texcoords,
                               int index,
                               const std::string& filepath) {
    size_t base = static_cast<size_t>(index) * 2;
    if (base + 1 >= texcoords.size()) {
        throw std::runtime_error("OBJ texcoord index out of range in " + filepath);
    }
    return glm::vec2(
        static_cast<float>(texcoords[base]),
        static_cast<float>(texcoords[base + 1]));
}

inline glm::vec3 read_normal(const std::vector<tinyobj::real_t>& normals,
                             int index,
                             const std::string& filepath) {
    size_t base = static_cast<size_t>(index) * 3;
    if (base + 2 >= normals.size()) {
        throw std::runtime_error("OBJ normal index out of range in " + filepath);
    }
    return glm::vec3(
        static_cast<float>(normals[base]),
        static_cast<float>(normals[base + 1]),
        static_cast<float>(normals[base + 2]));
}
} // namespace detail

inline ObjMeshData load_obj(const std::string& filepath) {
    tinyobj::ObjReaderConfig config;
    config.triangulate = true;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filepath, config)) {
        std::string err = reader.Error();
        if (err.empty()) {
            err = reader.Warning();
        }
        throw std::runtime_error(
            "Failed to parse OBJ file with tinyobjloader: " + filepath +
            (err.empty() ? "" : " | " + err));
    }

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();

    ObjMeshData data{};
    std::unordered_map<detail::VertexKey, uint32_t, detail::VertexKeyHash> vertex_lookup;
    bool has_input_normals = !attrib.normals.empty();

    for (const auto& shape : shapes) {
        for (const auto& idx : shape.mesh.indices) {
            if (idx.vertex_index < 0) {
                throw std::runtime_error("OBJ face references missing position data in " + filepath);
            }

            detail::VertexKey key{idx.vertex_index, idx.texcoord_index, idx.normal_index};

            auto it = vertex_lookup.find(key);
            if (it != vertex_lookup.end()) {
                data.indices.push_back(it->second);
                continue;
            }

            ObjVertex vertex{};
            vertex.position = detail::read_position(attrib.vertices, idx.vertex_index, filepath);

            if (idx.texcoord_index >= 0) {
                vertex.uv = detail::read_texcoord(attrib.texcoords, idx.texcoord_index, filepath);
            }

            if (idx.normal_index >= 0) {
                vertex.normal = detail::read_normal(attrib.normals, idx.normal_index, filepath);
            }

            uint32_t new_index = static_cast<uint32_t>(data.vertices.size());
            data.vertices.push_back(vertex);
            vertex_lookup.emplace(key, new_index);
            data.indices.push_back(new_index);
        }
    }

    if (!has_input_normals) {
        std::vector<glm::vec3> accum_normals(data.vertices.size(), glm::vec3(0.0f));
        for (size_t i = 0; i + 2 < data.indices.size(); i += 3) {
            auto i0 = data.indices[i];
            auto i1 = data.indices[i + 1];
            auto i2 = data.indices[i + 2];
            const glm::vec3& p0 = data.vertices[i0].position;
            const glm::vec3& p1 = data.vertices[i1].position;
            const glm::vec3& p2 = data.vertices[i2].position;
            glm::vec3 face_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
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

} // namespace rtr::utils
