#pragma once

#include <cstdint>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/component/pbpt_mesh.hpp"
#include "framework/core/scene.hpp"

namespace rtr::framework::integration {

struct PbptShapeRecord {
    std::string object_name{};
    std::string mesh_path{};
    glm::mat4 model{1.0f};
    glm::vec3 reflectance_rgb{0.7f, 0.7f, 0.7f};
    std::string material_id{};
};

struct PbptSceneRecord {
    std::vector<PbptShapeRecord> shapes{};
};

namespace detail {

inline std::string escape_xml(std::string value) {
    auto replace_all = [](std::string& text, const std::string& from, const std::string& to) {
        std::size_t pos = 0;
        while ((pos = text.find(from, pos)) != std::string::npos) {
            text.replace(pos, from.size(), to);
            pos += to.size();
        }
    };

    replace_all(value, "&", "&amp;");
    replace_all(value, "<", "&lt;");
    replace_all(value, ">", "&gt;");
    replace_all(value, "\"", "&quot;");
    replace_all(value, "\'", "&apos;");
    return value;
}

inline std::string rgb_key(const glm::vec3& value) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6)
        << value.x << "," << value.y << "," << value.z;
    return oss.str();
}

inline std::string serialize_rgb(const glm::vec3& value) {
    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6)
        << value.x << ", " << value.y << ", " << value.z;
    return oss.str();
}

inline std::string serialize_matrix_row_major(const glm::mat4& matrix) {
    std::ostringstream oss;
    oss << std::setprecision(9);
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            if (row != 0 || col != 0) {
                oss << ", ";
            }
            // GLM is column-major, convert to row-major ordering.
            oss << matrix[col][row];
        }
    }
    return oss.str();
}

} // namespace detail

inline PbptSceneRecord build_pbpt_scene_record(const core::Scene& scene) {
    PbptSceneRecord record{};

    std::unordered_map<std::string, std::string> material_ids{};

    for (const auto id : scene.scene_graph().active_nodes()) {
        const auto* go = scene.find_game_object(id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }

        const auto* mesh_renderer = go->get_component<component::MeshRenderer>();
        const auto* pbpt_mesh = go->get_component<component::PbptMesh>();
        if (mesh_renderer == nullptr || pbpt_mesh == nullptr) {
            continue;
        }
        if (!mesh_renderer->enabled() || !pbpt_mesh->enabled()) {
            continue;
        }

        const std::string& mesh_path = pbpt_mesh->mesh_path();
        if (mesh_path.empty()) {
            throw std::runtime_error("Pbpt export requires non-empty mesh_path.");
        }

        const glm::vec3 reflectance = pbpt_mesh->diffuse_bsdf().reflectance_rgb;
        const std::string reflectance_key = detail::rgb_key(reflectance);
        if (!material_ids.contains(reflectance_key)) {
            material_ids.emplace(
                reflectance_key,
                "mat_" + std::to_string(material_ids.size())
            );
        }

        std::string object_name = go->name();
        if (object_name.empty()) {
            object_name = "go_" + std::to_string(static_cast<std::uint64_t>(go->id()));
        }

        record.shapes.emplace_back(PbptShapeRecord{
            .object_name = std::move(object_name),
            .mesh_path = mesh_path,
            .model = scene.scene_graph().node(id).world_matrix(),
            .reflectance_rgb = reflectance,
            .material_id = material_ids.at(reflectance_key)
        });
    }

    return record;
}

inline std::string serialize_pbpt_scene_xml(const PbptSceneRecord& record) {
    struct MaterialEntry {
        std::string id{};
        glm::vec3 reflectance_rgb{0.7f, 0.7f, 0.7f};
    };

    std::unordered_map<std::string, std::string> material_ids{};
    std::vector<MaterialEntry> materials{};
    materials.reserve(record.shapes.size());

    for (const auto& shape : record.shapes) {
        const std::string key = detail::rgb_key(shape.reflectance_rgb);
        if (material_ids.contains(key)) {
            continue;
        }

        const std::string id = "mat_" + std::to_string(material_ids.size());
        material_ids.emplace(key, id);
        materials.emplace_back(MaterialEntry{
            .id = id,
            .reflectance_rgb = shape.reflectance_rgb
        });
    }

    std::ostringstream xml;
    xml << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
    xml << "<scene version=\"0.4.0\">\n";

    for (const auto& material : materials) {
        xml << "  <bsdf type=\"diffuse\" id=\"" << material.id << "\">\n";
        xml << "    <rgb name=\"reflectance\" value=\""
            << detail::serialize_rgb(material.reflectance_rgb)
            << "\"/>\n";
        xml << "  </bsdf>\n";
    }

    for (const auto& shape : record.shapes) {
        if (shape.mesh_path.empty()) {
            throw std::runtime_error("Pbpt export requires non-empty mesh_path.");
        }

        const std::string key = detail::rgb_key(shape.reflectance_rgb);
        const std::string& material_id = material_ids.at(key);

        xml << "  <shape type=\"obj\" id=\""
            << detail::escape_xml(shape.object_name)
            << "\">\n";
        xml << "    <string name=\"filename\" value=\""
            << detail::escape_xml(shape.mesh_path)
            << "\"/>\n";
        xml << "    <transform name=\"toWorld\">\n";
        xml << "      <matrix value=\""
            << detail::serialize_matrix_row_major(shape.model)
            << "\"/>\n";
        xml << "    </transform>\n";
        xml << "    <ref id=\"" << material_id << "\"/>\n";
        xml << "  </shape>\n";
    }

    xml << "</scene>\n";
    return xml.str();
}

} // namespace rtr::framework::integration
