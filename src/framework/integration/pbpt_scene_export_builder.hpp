#pragma once

#include <cstdint>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/component/pbpt_light.hpp"
#include "framework/component/pbpt_mesh.hpp"
#include "framework/core/camera.hpp"
#include "framework/core/scene.hpp"

namespace rtr::framework::integration {

struct PbptIntegratorRecord {
    std::string type{"path"};
    int max_depth{-1};
};

struct PbptSensorRecord {
    glm::mat4 to_world{1.0f};
    float fov_degrees{45.0f};
    float near_clip{0.1f};
    float far_clip{1000.0f};
    float focus_distance{1000.0f};
    int film_width{512};
    int film_height{512};
    int sample_count{4};
    std::string fov_axis{"smaller"};
};

struct PbptShapeRecord {
    std::string object_name{};
    std::string mesh_path{};
    glm::mat4 model{1.0f};
    component::PbptSpectrum reflectance_spectrum{
        component::make_constant_pbpt_spectrum(0.7f)
    };
    bool has_area_emitter{false};
    component::PbptSpectrum radiance_spectrum{};
    std::string material_id{};
};

struct PbptSceneRecord {
    std::optional<PbptIntegratorRecord> integrator{PbptIntegratorRecord{}};
    std::optional<PbptSensorRecord> sensor{};
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

inline std::string spectrum_key(const component::PbptSpectrum& spectrum) {
    return component::serialize_pbpt_spectrum(spectrum);
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
    record.integrator = PbptIntegratorRecord{};

    const auto* active_camera = scene.active_camera();
    if (const auto* perspective_camera = dynamic_cast<const core::PerspectiveCamera*>(active_camera);
        perspective_camera != nullptr) {
        PbptSensorRecord sensor{};
        sensor.to_world = perspective_camera->node().world_matrix();
        sensor.fov_degrees = perspective_camera->fov_degrees();
        sensor.near_clip = perspective_camera->near_bound();
        sensor.far_clip = perspective_camera->far_bound();
        record.sensor = sensor;
    }

    std::unordered_map<std::string, std::string> material_ids{};

    for (const auto id : scene.scene_graph().active_nodes()) {
        const auto* go = scene.find_game_object(id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }

        const auto* mesh_renderer = go->get_component<component::MeshRenderer>();
        const auto* pbpt_mesh = go->get_component<component::PbptMesh>();
        const auto* pbpt_light = go->get_component<component::PbptLight>();

        if (pbpt_light != nullptr && pbpt_mesh == nullptr) {
            throw std::runtime_error(
                "PbptLight requires PbptMesh on the same GameObject for export."
            );
        }

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

        const component::PbptSpectrum& reflectance = pbpt_mesh->reflectance_spectrum();
        component::validate_pbpt_spectrum(reflectance, "PbptMesh.reflectance_spectrum");
        const std::string reflectance_key = detail::spectrum_key(reflectance);
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
            .reflectance_spectrum = reflectance,
            .has_area_emitter = pbpt_light != nullptr && pbpt_light->enabled(),
            .radiance_spectrum =
                (pbpt_light != nullptr && pbpt_light->enabled())
                    ? pbpt_light->area_emitter().radiance_spectrum
                    : component::PbptSpectrum{},
            .material_id = material_ids.at(reflectance_key)
        });
    }

    return record;
}

inline std::string serialize_pbpt_scene_xml(const PbptSceneRecord& record) {
    struct MaterialEntry {
        std::string id{};
        component::PbptSpectrum reflectance_spectrum{
            component::make_constant_pbpt_spectrum(0.7f)
        };
    };

    std::unordered_map<std::string, std::string> material_ids{};
    std::vector<MaterialEntry> materials{};
    materials.reserve(record.shapes.size());

    for (const auto& shape : record.shapes) {
        component::validate_pbpt_spectrum(shape.reflectance_spectrum, "shape.reflectance_spectrum");
        const std::string key = detail::spectrum_key(shape.reflectance_spectrum);
        if (material_ids.contains(key)) {
            continue;
        }

        const std::string id = "mat_" + std::to_string(material_ids.size());
        material_ids.emplace(key, id);
        materials.emplace_back(MaterialEntry{
            .id = id,
            .reflectance_spectrum = shape.reflectance_spectrum
        });
    }

    std::ostringstream xml;
    xml << "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\n";
    xml << "<scene version=\"0.4.0\">\n";

    if (record.integrator.has_value()) {
        const auto& integrator = record.integrator.value();
        xml << "  <integrator type=\""
            << detail::escape_xml(integrator.type)
            << "\">\n";
        xml << "    <integer name=\"maxDepth\" value=\"" << integrator.max_depth << "\"/>\n";
        xml << "  </integrator>\n";
    }

    if (record.sensor.has_value()) {
        const auto& sensor = record.sensor.value();
        if (sensor.film_width <= 0 || sensor.film_height <= 0) {
            throw std::runtime_error("Pbpt sensor film size must be positive.");
        }
        if (sensor.sample_count <= 0) {
            throw std::runtime_error("Pbpt sensor sample_count must be positive.");
        }

        xml << "  <sensor type=\"perspective\">\n";
        xml << "    <string name=\"fovAxis\" value=\""
            << detail::escape_xml(sensor.fov_axis)
            << "\"/>\n";
        xml << "    <float name=\"nearClip\" value=\"" << sensor.near_clip << "\"/>\n";
        xml << "    <float name=\"farClip\" value=\"" << sensor.far_clip << "\"/>\n";
        xml << "    <float name=\"focusDistance\" value=\"" << sensor.focus_distance << "\"/>\n";
        xml << "    <transform name=\"toWorld\">\n";
        xml << "      <matrix value=\""
            << detail::serialize_matrix_row_major(sensor.to_world)
            << "\"/>\n";
        xml << "    </transform>\n";
        xml << "    <float name=\"fov\" value=\"" << sensor.fov_degrees << "\"/>\n";
        xml << "    <sampler type=\"ldsampler\">\n";
        xml << "      <integer name=\"sampleCount\" value=\"" << sensor.sample_count << "\"/>\n";
        xml << "    </sampler>\n";
        xml << "    <film type=\"hdrfilm\">\n";
        xml << "      <integer name=\"width\" value=\"" << sensor.film_width << "\"/>\n";
        xml << "      <integer name=\"height\" value=\"" << sensor.film_height << "\"/>\n";
        xml << "      <rfilter type=\"gaussian\"/>\n";
        xml << "    </film>\n";
        xml << "  </sensor>\n";
    }

    for (const auto& material : materials) {
        xml << "  <bsdf type=\"diffuse\" id=\"" << material.id << "\">\n";
        xml << "    <spectrum name=\"reflectance\" value=\""
            << detail::escape_xml(component::serialize_pbpt_spectrum(material.reflectance_spectrum))
            << "\"/>\n";
        xml << "  </bsdf>\n";
    }

    for (const auto& shape : record.shapes) {
        if (shape.mesh_path.empty()) {
            throw std::runtime_error("Pbpt export requires non-empty mesh_path.");
        }

        const std::string key = detail::spectrum_key(shape.reflectance_spectrum);
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
        if (shape.has_area_emitter) {
            component::validate_pbpt_spectrum(shape.radiance_spectrum, "shape.radiance_spectrum");
            xml << "    <emitter type=\"area\">\n";
            xml << "      <spectrum name=\"radiance\" value=\""
                << detail::escape_xml(component::serialize_pbpt_spectrum(shape.radiance_spectrum))
                << "\"/>\n";
            xml << "    </emitter>\n";
        }
        xml << "  </shape>\n";
    }

    xml << "</scene>\n";
    return xml.str();
}

} // namespace rtr::framework::integration
