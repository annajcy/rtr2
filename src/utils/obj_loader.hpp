#pragma once

#include <array>
#include <cctype>
#include <cstdint>
#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

namespace rtr::utils {

struct ObjVertex {
    glm::vec3 position{0.0f};
    glm::vec3 color{1.0f};
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

inline int to_index(int idx, int count) {
    // OBJ indices are 1-based; negative indices are relative to the end.
    if (idx > 0) {
        return idx - 1;
    } else if (idx < 0) {
        return count + idx;
    } else {
        return -1;
    }
}

inline std::vector<std::string> split_whitespace(const std::string& line) {
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
        tokens.push_back(token);
    }
    return tokens;
}

inline VertexKey parse_face_token(const std::string& token) {
    VertexKey key{};
    std::array<int*, 3> targets = {&key.pos, &key.tex, &key.norm};

    size_t start = 0;
    int field = 0;
    while (start <= token.size() && field < 3) {
        size_t slash = token.find('/', start);
        std::string part = token.substr(start, slash == std::string::npos ? std::string::npos : slash - start);
        if (!part.empty()) {
            *targets[field] = std::stoi(part);
        }
        if (slash == std::string::npos) {
            break;
        }
        start = slash + 1;
        ++field;
    }
    return key;
}

inline glm::vec3 parse_vec3(const std::vector<std::string>& tokens, size_t start) {
    return glm::vec3{
        std::stof(tokens[start]),
        std::stof(tokens[start + 1]),
        std::stof(tokens[start + 2])
    };
}

inline glm::vec2 parse_vec2(const std::vector<std::string>& tokens, size_t start) {
    return glm::vec2{
        std::stof(tokens[start]),
        std::stof(tokens[start + 1])
    };
}
} // namespace detail

inline ObjMeshData load_obj(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Failed to open OBJ file: " + filepath);
    }

    std::vector<glm::vec3> positions;
    std::vector<glm::vec3> position_colors;
    std::vector<glm::vec2> texcoords;
    std::vector<glm::vec3> normals;

    std::vector<ObjVertex> vertices;
    std::vector<uint32_t> indices;
    std::unordered_map<detail::VertexKey, uint32_t, detail::VertexKeyHash> vertex_lookup;
    bool has_input_normals = false;

    std::string line;
    while (std::getline(file, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }

        auto tokens = detail::split_whitespace(line);
        if (tokens.empty()) continue;

        if (tokens[0] == "v" && tokens.size() >= 4) {
            positions.push_back(detail::parse_vec3(tokens, 1));
            // Optional per-vertex color appended on the v line.
            if (tokens.size() >= 7) {
                position_colors.emplace_back(
                    std::stof(tokens[4]),
                    std::stof(tokens[5]),
                    std::stof(tokens[6])
                );
            } else {
                position_colors.emplace_back(1.0f, 1.0f, 1.0f);
            }
        } else if (tokens[0] == "vt" && tokens.size() >= 3) {
            texcoords.push_back(detail::parse_vec2(tokens, 1));
        } else if (tokens[0] == "vn" && tokens.size() >= 4) {
            normals.push_back(detail::parse_vec3(tokens, 1));
            has_input_normals = true;
        } else if (tokens[0] == "f" && tokens.size() >= 4) {
            // Triangulate faces by fan method if there are more than 3 vertices.
            std::vector<detail::VertexKey> face_keys;
            for (size_t i = 1; i < tokens.size(); ++i) {
                face_keys.push_back(detail::parse_face_token(tokens[i]));
            }

            for (size_t tri = 1; tri + 1 < face_keys.size(); ++tri) {
                const detail::VertexKey tri_keys[3] = {
                    face_keys[0], face_keys[tri], face_keys[tri + 1]
                };

                for (const auto& key_raw : tri_keys) {
                    detail::VertexKey key = key_raw;
                    key.pos = detail::to_index(key.pos, static_cast<int>(positions.size()));
                    key.tex = detail::to_index(key.tex, static_cast<int>(texcoords.size()));
                    key.norm = detail::to_index(key.norm, static_cast<int>(normals.size()));

                    if (key.pos < 0 || key.pos >= static_cast<int>(positions.size())) {
                        throw std::runtime_error("OBJ face references missing position data in " + filepath);
                    }

                    auto it = vertex_lookup.find(key);
                    if (it != vertex_lookup.end()) {
                        indices.push_back(it->second);
                        continue;
                    }

                    ObjVertex vert{};
                    if (key.pos >= 0 && key.pos < static_cast<int>(positions.size())) {
                        vert.position = positions[key.pos];
                        vert.color = position_colors[key.pos];
                    }
                    if (key.tex >= 0 && key.tex < static_cast<int>(texcoords.size())) {
                        vert.uv = texcoords[key.tex];
                    }
                    if (key.norm >= 0 && key.norm < static_cast<int>(normals.size())) {
                        vert.normal = normals[key.norm];
                    }

                    uint32_t new_index = static_cast<uint32_t>(vertices.size());
                    vertices.push_back(vert);
                    vertex_lookup.emplace(key, new_index);
                    indices.push_back(new_index);
                }
            }
        }
    }

    // Generate smooth normals if source mesh had none.
    if (!has_input_normals) {
        std::vector<glm::vec3> accum_normals(vertices.size(), glm::vec3(0.0f));
        for (size_t i = 0; i + 2 < indices.size(); i += 3) {
            auto i0 = indices[i];
            auto i1 = indices[i + 1];
            auto i2 = indices[i + 2];
            const glm::vec3& p0 = vertices[i0].position;
            const glm::vec3& p1 = vertices[i1].position;
            const glm::vec3& p2 = vertices[i2].position;
            glm::vec3 face_normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));
            accum_normals[i0] += face_normal;
            accum_normals[i1] += face_normal;
            accum_normals[i2] += face_normal;
        }
        for (size_t v = 0; v < vertices.size(); ++v) {
            if (glm::length(accum_normals[v]) > 0.0f) {
                vertices[v].normal = glm::normalize(accum_normals[v]);
            } else {
                vertices[v].normal = glm::vec3(0.0f, 1.0f, 0.0f);
            }
        }
    }

    ObjMeshData data;
    data.vertices = std::move(vertices);
    data.indices = std::move(indices);
    return data;
}

} // namespace rtr::utils
