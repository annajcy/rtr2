#pragma once

#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <variant>

#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/obj_io.hpp"

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
    resource::MeshHandle mesh_handle{};
    glm::mat4 model{1.0f};
    component::PbptReflectance reflectance{
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

inline std::string rgb_value_string(const component::PbptRgb& rgb) {
    component::validate_pbpt_rgb(rgb, "PbptShapeRecord.reflectance_rgb");
    std::ostringstream oss;
    oss << std::setprecision(6) << rgb.r << " " << rgb.g << " " << rgb.b;
    return oss.str();
}

inline std::string reflectance_key(const component::PbptReflectance& reflectance) {
    return std::visit(
        [](const auto& payload) -> std::string {
            using PayloadT = std::decay_t<decltype(payload)>;
            if constexpr (std::is_same_v<PayloadT, component::PbptSpectrum>) {
                return "spectrum:" + spectrum_key(payload);
            } else {
                return "rgb:" + rgb_value_string(payload);
            }
        },
        reflectance
    );
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

inline std::filesystem::path resolve_meshes_output_dir(const std::string& scene_xml_path) {
    if (scene_xml_path.empty()) {
        throw std::invalid_argument("scene_xml_path must not be empty.");
    }

    const auto abs_xml = std::filesystem::absolute(std::filesystem::path(scene_xml_path));
    const auto xml_parent = abs_xml.parent_path();
    if (xml_parent.empty()) {
        throw std::runtime_error("scene_xml_path must have a parent directory.");
    }
    return xml_parent / "meshes";
}

inline std::string mesh_file_name(resource::MeshHandle handle) {
    return "mesh_" + std::to_string(handle.value) + ".obj";
}

inline std::filesystem::path make_mesh_relative_xml_path(resource::MeshHandle handle) {
    const std::filesystem::path rel = std::filesystem::path("meshes") / mesh_file_name(handle);
    if (rel.parent_path() != std::filesystem::path("meshes") || rel.extension() != ".obj") {
        throw std::runtime_error("PBPT mesh XML path contract violation.");
    }
    return rel;
}

} // namespace detail

inline PbptSceneRecord build_pbpt_scene_record(
    const core::Scene& scene,
    resource::ResourceManager& resources
) {
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

        const resource::MeshHandle mesh_handle = pbpt_mesh->mesh_handle();
        if (!mesh_handle.is_valid() || !resources.mesh_alive(mesh_handle)) {
            throw std::runtime_error("Pbpt export requires valid and alive mesh handle.");
        }

        const component::PbptReflectance& reflectance = pbpt_mesh->reflectance();
        if (const auto* spectrum = std::get_if<component::PbptSpectrum>(&reflectance)) {
            component::validate_pbpt_spectrum(*spectrum, "PbptMesh.reflectance_spectrum");
        } else {
            component::validate_pbpt_rgb(std::get<component::PbptRgb>(reflectance), "PbptMesh.reflectance_rgb");
        }
        const std::string reflectance_key = detail::reflectance_key(reflectance);
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
            .mesh_handle = mesh_handle,
            .model = scene.scene_graph().node(id).world_matrix(),
            .reflectance = reflectance,
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

inline std::string serialize_pbpt_scene_xml(
    const PbptSceneRecord& record,
    resource::ResourceManager& resources,
    const std::string& scene_xml_path
) {
    struct MaterialEntry {
        std::string id{};
        component::PbptReflectance reflectance{
            component::make_constant_pbpt_spectrum(0.7f)
        };
    };

    std::unordered_map<std::string, std::string> material_ids{};
    std::vector<MaterialEntry> materials{};
    materials.reserve(record.shapes.size());

    for (const auto& shape : record.shapes) {
        if (const auto* spectrum = std::get_if<component::PbptSpectrum>(&shape.reflectance)) {
            component::validate_pbpt_spectrum(*spectrum, "shape.reflectance_spectrum");
        } else {
            component::validate_pbpt_rgb(std::get<component::PbptRgb>(shape.reflectance), "shape.reflectance_rgb");
        }
        const std::string key = detail::reflectance_key(shape.reflectance);
        if (material_ids.contains(key)) {
            continue;
        }

        const std::string id = "mat_" + std::to_string(material_ids.size());
        material_ids.emplace(key, id);
        materials.emplace_back(MaterialEntry{
            .id = id,
            .reflectance = shape.reflectance
        });
    }

    const std::filesystem::path mesh_output_dir = detail::resolve_meshes_output_dir(scene_xml_path);
    std::filesystem::create_directories(mesh_output_dir);

    std::unordered_map<resource::MeshHandle, std::string> mesh_relative_path_by_handle{};
    for (const auto& shape : record.shapes) {
        if (!shape.mesh_handle.is_valid() || !resources.mesh_alive(shape.mesh_handle)) {
            throw std::runtime_error("Pbpt export requires valid and alive mesh handle.");
        }

        if (mesh_relative_path_by_handle.contains(shape.mesh_handle)) {
            continue;
        }

        const std::string filename = detail::mesh_file_name(shape.mesh_handle);
        const std::filesystem::path abs_mesh_path = mesh_output_dir / filename;
        const std::filesystem::path rel_mesh_path = detail::make_mesh_relative_xml_path(shape.mesh_handle);

        utils::write_obj_to_path(resources.mesh_cpu(shape.mesh_handle), abs_mesh_path.string());
        mesh_relative_path_by_handle.emplace(shape.mesh_handle, rel_mesh_path.generic_string());
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
        if (const auto* spectrum = std::get_if<component::PbptSpectrum>(&material.reflectance)) {
            xml << "    <spectrum name=\"reflectance\" value=\""
                << detail::escape_xml(component::serialize_pbpt_spectrum(*spectrum))
                << "\"/>\n";
        } else {
            xml << "    <rgb name=\"reflectance\" value=\""
                << detail::escape_xml(
                       detail::rgb_value_string(std::get<component::PbptRgb>(material.reflectance))
                   )
                << "\"/>\n";
        }
        xml << "  </bsdf>\n";
    }

    for (const auto& shape : record.shapes) {
        if (!shape.mesh_handle.is_valid()) {
            throw std::runtime_error("Pbpt export requires valid mesh_handle.");
        }

        const std::string key = detail::reflectance_key(shape.reflectance);
        const std::string& material_id = material_ids.at(key);
        const std::string& mesh_rel_path = mesh_relative_path_by_handle.at(shape.mesh_handle);

        xml << "  <shape type=\"obj\" id=\""
            << detail::escape_xml(shape.object_name)
            << "\">\n";
        xml << "    <string name=\"filename\" value=\""
            << detail::escape_xml(mesh_rel_path)
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
