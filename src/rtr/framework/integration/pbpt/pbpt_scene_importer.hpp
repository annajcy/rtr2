#pragma once

#include <pbpt/math/math.h>

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <filesystem>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include <pugixml.hpp>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_export_builder.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_state.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::integration {

inline std::shared_ptr<spdlog::logger> pbpt_import_logger() {
    return utils::get_logger("framework.integration.pbpt.import");
}

struct PbptImportOptions {
    bool require_supported_cbox_subset{true};
    // When set, importer attaches FreeLookCameraController to the imported active camera.
    const system::input::InputState* free_look_input_state{nullptr};
};

struct PbptImportResult {
    std::size_t imported_shape_count{0};
    std::size_t imported_light_shape_count{0};
    std::unordered_map<std::string, core::GameObjectId> imported_game_object_id_by_name{};
    std::optional<PbptIntegratorRecord> integrator{};
    std::optional<PbptSensorRecord> sensor{};
};

struct PbptSceneLocation {
    std::string scene_root_rel_to_resource_dir{};
    std::string xml_filename{};
};

inline PbptSceneLocation make_pbpt_scene_location(
    std::string scene_root_rel_to_resource_dir,
    std::string xml_filename
) {
    return PbptSceneLocation{
        .scene_root_rel_to_resource_dir = std::move(scene_root_rel_to_resource_dir),
        .xml_filename = std::move(xml_filename)
    };
}

namespace detail {

inline std::string trim_copy(std::string text) {
    auto is_space = [](unsigned char c) {
        return std::isspace(c) != 0;
    };

    const auto begin = std::find_if_not(text.begin(), text.end(), is_space);
    if (begin == text.end()) {
        return {};
    }

    const auto end = std::find_if_not(text.rbegin(), text.rend(), is_space).base();
    return std::string(begin, end);
}

inline std::vector<float> parse_float_list(std::string text, std::string_view field_name) {
    for (char& c : text) {
        if (c == ',') {
            c = ' ';
        }
    }

    std::stringstream ss(text);
    std::vector<float> values{};
    float value = 0.0f;
    while (ss >> value) {
        values.emplace_back(value);
    }

    if (values.empty()) {
        throw std::runtime_error(std::string(field_name) + " has no numeric values.");
    }

    return values;
}

inline pbpt::math::vec3 parse_vec3_csv(const std::string& text, std::string_view field_name) {
    const auto values = parse_float_list(text, field_name);
    if (values.size() != 3u) {
        throw std::runtime_error(std::string(field_name) + " must have exactly 3 values.");
    }
    return {values[0], values[1], values[2]};
}

inline component::PbptSpectrum parse_pbpt_spectrum(
    const std::string& text,
    std::string_view field_name
) {
    component::PbptSpectrum spectrum{};
    std::stringstream ss(text);
    std::string token;

    while (std::getline(ss, token, ',')) {
        token = trim_copy(std::move(token));
        if (token.empty()) {
            continue;
        }

        const std::size_t sep = token.find(':');
        if (sep == std::string::npos) {
            throw std::runtime_error(std::string(field_name) + " token must be formatted as lambda:value.");
        }

        const std::string lambda_token = trim_copy(token.substr(0, sep));
        const std::string value_token = trim_copy(token.substr(sep + 1));
        if (lambda_token.empty() || value_token.empty()) {
            throw std::runtime_error(std::string(field_name) + " token must be formatted as lambda:value.");
        }

        float lambda_nm = 0.0f;
        float value = 0.0f;
        try {
            lambda_nm = std::stof(lambda_token);
            value = std::stof(value_token);
        } catch (const std::exception&) {
            throw std::runtime_error(std::string(field_name) + " contains non-numeric spectrum value.");
        }

        spectrum.emplace_back(component::PbptSpectrumPoint{lambda_nm, value});
    }

    component::validate_pbpt_spectrum(spectrum, field_name);
    return spectrum;
}

inline component::PbptRgb parse_pbpt_rgb(
    const std::string& text,
    std::string_view field_name
) {
    const auto values = parse_float_list(text, field_name);
    if (values.size() != 3u) {
        throw std::runtime_error(std::string(field_name) + " must have exactly 3 values.");
    }
    component::PbptRgb rgb{
        .r = values[0],
        .g = values[1],
        .b = values[2]
    };
    component::validate_pbpt_rgb(rgb, field_name);
    return rgb;
}

inline component::PbptRgb parse_bsdf_reflectance(
    const pugi::xml_node& bsdf_node
) {
    for (const auto& child : bsdf_node.children()) {
        const std::string_view tag = child.name();
        if (tag != "spectrum" && tag != "rgb") {
            continue;
        }
        if (std::string_view(child.attribute("name").value()) != "reflectance") {
            continue;
        }

        const std::string value = child.attribute("value").value();
        if (value.empty()) {
            continue;
        }
        if (tag == "spectrum") {
            return pbpt_spectrum_to_rgb(parse_pbpt_spectrum(value, "bsdf.reflectance"));
        }
        return parse_pbpt_rgb(value, "bsdf.reflectance");
    }

    return component::PbptRgb{
        .r = 0.7f,
        .g = 0.7f,
        .b = 0.7f
    };
}

inline pbpt::math::mat4 parse_matrix_row_major(const std::string& text, std::string_view field_name) {
    const auto values = parse_float_list(text, field_name);
    if (values.size() != 16u) {
        throw std::runtime_error(std::string(field_name) + " must contain exactly 16 float values.");
    }

    pbpt::math::mat4 matrix{1.0f};
    std::size_t idx = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[row][col] = values[idx++];
        }
    }
    return matrix;
}

inline pugi::xml_node find_named_child(
    const pugi::xml_node& node,
    const char* tag,
    std::string_view name
) {
    for (const auto& child : node.children(tag)) {
        if (std::string_view(child.attribute("name").value()) == name) {
            return child;
        }
    }
    return {};
}

inline std::optional<float> find_float_property(
    const pugi::xml_node& node,
    std::string_view name
) {
    const auto child = find_named_child(node, "float", name);
    if (!child) {
        return std::nullopt;
    }
    return child.attribute("value").as_float();
}

inline std::optional<int> find_integer_property(
    const pugi::xml_node& node,
    std::string_view name
) {
    const auto child = find_named_child(node, "integer", name);
    if (!child) {
        return std::nullopt;
    }
    return child.attribute("value").as_int();
}

inline std::optional<std::string> find_string_property(
    const pugi::xml_node& node,
    std::string_view name
) {
    const auto child = find_named_child(node, "string", name);
    if (!child) {
        return std::nullopt;
    }
    const std::string value = child.attribute("value").value();
    if (value.empty()) {
        return std::nullopt;
    }
    return value;
}

inline pbpt::math::mat4 parse_shape_transform(
    const pugi::xml_node& transform_node,
    bool strict_mode
) {
    pbpt::math::mat4 transform{1.0f};

    for (const auto& child : transform_node.children()) {
        const std::string_view tag = child.name();
        if (tag == "matrix") {
            const std::string value = child.attribute("value").value();
            if (value.empty()) {
                throw std::runtime_error("shape transform matrix is missing value.");
            }
            transform = transform * parse_matrix_row_major(value, "shape.transform.matrix");
            continue;
        }

        if (tag == "translate") {
            const float x = child.attribute("x").as_float(0.0f);
            const float y = child.attribute("y").as_float(0.0f);
            const float z = child.attribute("z").as_float(0.0f);
            transform = transform * pbpt::math::translate(pbpt::math::mat4{1.0f}, pbpt::math::vec3{x, y, z});
            continue;
        }

        if (strict_mode) {
            throw std::runtime_error("Unsupported shape transform element: " + std::string(tag));
        }
    }

    return transform;
}

inline pbpt::math::mat4 parse_sensor_to_world(
    const pugi::xml_node& transform_node,
    bool strict_mode
) {
    bool has_look_at = false;
    bool has_matrix = false;
    pbpt::math::mat4 to_world{1.0f};

    for (const auto& child : transform_node.children()) {
        const std::string_view tag = child.name();
        if (tag == "lookAt") {
            if (has_matrix) {
                throw std::runtime_error("Sensor transform cannot contain both lookAt and matrix.");
            }

            const std::string origin_text = child.attribute("origin").value();
            const std::string target_text = child.attribute("target").value();
            const std::string up_text = child.attribute("up").value();
            if (origin_text.empty() || target_text.empty() || up_text.empty()) {
                throw std::runtime_error("Sensor lookAt must provide origin/target/up.");
            }

            const pbpt::math::vec3 origin = parse_vec3_csv(origin_text, "sensor.lookAt.origin");
            const pbpt::math::vec3 target = parse_vec3_csv(target_text, "sensor.lookAt.target");
            const pbpt::math::vec3 up = parse_vec3_csv(up_text, "sensor.lookAt.up");
            const pbpt::math::vec3 forward = pbpt::math::normalize(target - origin);
            if (pbpt::math::length(forward) < 1e-6f) {
                throw std::runtime_error("sensor.lookAt origin and target must be different.");
            }

            pbpt::math::vec3 up_dir = pbpt::math::normalize(up);
            if (pbpt::math::length(up_dir) < 1e-6f) {
                throw std::runtime_error("sensor.lookAt up vector must be non-zero.");
            }

            // RTR camera convention: local -Z is front, +Y is up.
            const pbpt::math::vec3 right = pbpt::math::normalize(pbpt::math::cross(forward, up_dir));
            if (pbpt::math::length(right) < 1e-6f) {
                throw std::runtime_error("sensor.lookAt up vector must not be parallel to view direction.");
            }

            up_dir = pbpt::math::normalize(pbpt::math::cross(right, forward));
            // Matrix[row][col] storage: put camera basis in columns and translation in column 3.
            to_world = pbpt::math::mat4{1.0f};
            to_world[0][0] = right.x();
            to_world[1][0] = right.y();
            to_world[2][0] = right.z();

            to_world[0][1] = up_dir.x();
            to_world[1][1] = up_dir.y();
            to_world[2][1] = up_dir.z();

            // Local +Z points backwards when local -Z is camera forward.
            to_world[0][2] = -forward.x();
            to_world[1][2] = -forward.y();
            to_world[2][2] = -forward.z();

            to_world[0][3] = origin.x();
            to_world[1][3] = origin.y();
            to_world[2][3] = origin.z();
            has_look_at = true;
            continue;
        }

        if (tag == "matrix") {
            if (has_look_at) {
                throw std::runtime_error("Sensor transform cannot contain both lookAt and matrix.");
            }

            const std::string value = child.attribute("value").value();
            if (value.empty()) {
                throw std::runtime_error("sensor transform matrix is missing value.");
            }
            to_world = parse_matrix_row_major(value, "sensor.transform.matrix");
            has_matrix = true;
            continue;
        }

        if (strict_mode) {
            throw std::runtime_error("Unsupported sensor transform element: " + std::string(tag));
        }
    }

    return to_world;
}

inline std::string default_object_name(
    const pugi::xml_node& shape_node,
    const std::filesystem::path& mesh_path,
    std::size_t fallback_index
) {
    const std::string id_attr = shape_node.attribute("id").value();
    if (!id_attr.empty()) {
        return id_attr;
    }

    const std::string stem = mesh_path.stem().string();
    if (!stem.empty()) {
        return stem;
    }

    return "shape_" + std::to_string(fallback_index);
}

inline void register_imported_game_object(
    PbptImportResult& result,
    const std::string& name,
    core::GameObjectId id
) {
    auto [_, inserted] = result.imported_game_object_id_by_name.emplace(name, id);
    if (!inserted) {
        throw std::runtime_error("Duplicate imported game object name: " + name);
    }
}

inline void validate_scene_location(const PbptSceneLocation& location) {
    if (location.xml_filename.empty()) {
        pbpt_import_logger()->error("validate_scene_location failed: xml_filename is empty.");
        throw std::invalid_argument("xml_filename must not be empty.");
    }

    const std::filesystem::path xml_filename_path(location.xml_filename);
    if (xml_filename_path.is_absolute() || xml_filename_path.has_parent_path()) {
        pbpt_import_logger()->error(
            "validate_scene_location failed: xml_filename '{}' contains path separator or is absolute.",
            location.xml_filename
        );
        throw std::invalid_argument("xml_filename must be a filename without path separator.");
    }
}

inline std::filesystem::path validate_and_normalize_mesh_path(
    const std::filesystem::path& mesh_path
) {
    if (mesh_path.empty()) {
        pbpt_import_logger()->error("validate_and_normalize_mesh_path failed: mesh path is empty.");
        throw std::runtime_error("obj filename must not be empty.");
    }
    if (mesh_path.is_absolute()) {
        pbpt_import_logger()->error(
            "validate_and_normalize_mesh_path failed: mesh path '{}' is absolute.",
            mesh_path.string()
        );
        throw std::runtime_error("obj filename must be a relative path under meshes/.");
    }

    const std::filesystem::path normalized = mesh_path.lexically_normal();
    if (normalized.empty()) {
        pbpt_import_logger()->error("validate_and_normalize_mesh_path failed: normalized path is empty.");
        throw std::runtime_error("obj filename must not be empty.");
    }
    for (const auto& part : normalized) {
        if (part == "..") {
            pbpt_import_logger()->error(
                "validate_and_normalize_mesh_path failed: mesh path '{}' uses parent traversal.",
                normalized.string()
            );
            throw std::runtime_error("obj filename must not use parent directory traversal.");
        }
    }

    auto it = normalized.begin();
    if (it == normalized.end() || *it != "meshes") {
        pbpt_import_logger()->error(
            "validate_and_normalize_mesh_path failed: mesh path '{}' is outside meshes/.",
            normalized.string()
        );
        throw std::runtime_error("obj filename must resolve under meshes/ directory.");
    }

    return normalized;
}

} // namespace detail

inline PbptImportResult import_pbpt_scene_xml_to_scene(
    const PbptSceneLocation& location,
    core::Scene& scene,
    resource::ResourceManager& resources,
    const PbptImportOptions& options = {}
) {
    auto log = pbpt_import_logger();
    detail::validate_scene_location(location);
    const std::filesystem::path xml_abs_path = resources.resource_root_dir() / location.scene_root_rel_to_resource_dir / location.xml_filename;
    log->info("Importing PBPT scene XML from '{}'.", xml_abs_path.string());
    pugi::xml_document doc;
    const pugi::xml_parse_result parse_result = doc.load_file(xml_abs_path.string().c_str());
    if (!parse_result) {
        log->error("PBPT XML load error for '{}': {}", xml_abs_path.string(), parse_result.description());
        throw std::runtime_error(std::string("XML load error: ") + parse_result.description());
    }

    const pugi::xml_node root = doc.child("scene");
    if (!root) {
        log->error("PBPT XML import failed: root node <scene> is missing.");
        throw std::runtime_error("XML root node <scene> is missing.");
    }

    PbptImportResult result{};
    std::unordered_map<std::string, component::PbptRgb> reflectance_by_bsdf_id{};

    for (const auto& bsdf_node : root.children("bsdf")) {
        const std::string type = bsdf_node.attribute("type").value();
        const std::string id = bsdf_node.attribute("id").value();
        if (id.empty()) {
            log->error("PBPT XML import failed: bsdf node missing id.");
            throw std::runtime_error("bsdf id is required.");
        }

        if (type != "diffuse") {
            if (options.require_supported_cbox_subset) {
                log->error("PBPT XML import failed: unsupported bsdf type '{}'.", type);
                throw std::runtime_error("Unsupported bsdf type: " + type);
            }
            continue;
        }

        reflectance_by_bsdf_id[id] = detail::parse_bsdf_reflectance(bsdf_node);
    }

    if (const auto integrator_node = root.child("integrator"); integrator_node) {
        PbptIntegratorRecord integrator{};
        integrator.type = integrator_node.attribute("type").value();
        if (integrator.type.empty()) {
            log->error("PBPT XML import failed: integrator type is empty.");
            throw std::runtime_error("integrator type is required.");
        }
        if (integrator.type != "path" && options.require_supported_cbox_subset) {
            log->error("PBPT XML import failed: unsupported integrator type '{}'.", integrator.type);
            throw std::runtime_error("Unsupported integrator type: " + integrator.type);
        }

        if (const auto max_depth = detail::find_integer_property(integrator_node, "maxDepth");
            max_depth.has_value()) {
            integrator.max_depth = max_depth.value();
        }

        result.integrator = integrator;
    }

    if (const auto sensor_node = root.child("sensor"); sensor_node) {
        const std::string type = sensor_node.attribute("type").value();
        if (type != "perspective" && options.require_supported_cbox_subset) {
            log->error("PBPT XML import failed: unsupported sensor type '{}'.", type);
            throw std::runtime_error("Unsupported sensor type: " + type);
        }

        PbptSensorRecord sensor{};
        if (const auto fov_axis = detail::find_string_property(sensor_node, "fovAxis"); fov_axis.has_value()) {
            sensor.fov_axis = fov_axis.value();
        }
        if (const auto near_clip = detail::find_float_property(sensor_node, "nearClip"); near_clip.has_value()) {
            sensor.near_clip = near_clip.value();
        }
        if (const auto far_clip = detail::find_float_property(sensor_node, "farClip"); far_clip.has_value()) {
            sensor.far_clip = far_clip.value();
        }
        if (const auto focus_distance = detail::find_float_property(sensor_node, "focusDistance");
            focus_distance.has_value()) {
            sensor.focus_distance = focus_distance.value();
        }
        if (const auto fov = detail::find_float_property(sensor_node, "fov"); fov.has_value()) {
            sensor.fov_degrees = fov.value();
        }

        if (const auto sampler_node = sensor_node.child("sampler"); sampler_node) {
            if (const auto sample_count = detail::find_integer_property(sampler_node, "sampleCount");
                sample_count.has_value()) {
                sensor.sample_count = sample_count.value();
            }
        }

        if (const auto film_node = sensor_node.child("film"); film_node) {
            if (const auto width = detail::find_integer_property(film_node, "width"); width.has_value()) {
                sensor.film_width = width.value();
            }
            if (const auto height = detail::find_integer_property(film_node, "height"); height.has_value()) {
                sensor.film_height = height.value();
            }
        }

        if (const auto transform_node = sensor_node.child("transform"); transform_node) {
            sensor.to_world = detail::parse_sensor_to_world(
                transform_node,
                options.require_supported_cbox_subset
            );
        }

        auto& camera_go = scene.create_game_object("pbpt_camera");
        auto& camera = scene.camera_manager().create_perspective_camera(camera_go.id());
        camera.near_bound() = sensor.near_clip;
        camera.far_bound() = sensor.far_clip;
        camera.fov_degrees() = sensor.fov_degrees;
        camera.set_aspect_ratio(static_cast<float>(sensor.film_width) /
                                static_cast<float>(std::max(1, sensor.film_height)));
        camera_go.node().set_local_model_matrix(sensor.to_world);
        (void)scene.set_active_camera(camera_go.id());
        detail::register_imported_game_object(result, camera_go.name(), camera_go.id());
        if (options.free_look_input_state != nullptr) {
            (void)camera_go.add_component<component::FreeLookCameraController>(
                options.free_look_input_state,
                &scene.camera_manager()
            );
        }

        result.sensor = sensor;
    }

    std::size_t shape_index = 0;
    for (const auto& shape_node : root.children("shape")) {
        const std::string type = shape_node.attribute("type").value();
        if (type != "obj") {
            if (options.require_supported_cbox_subset) {
                log->error("PBPT XML import failed: unsupported shape type '{}'.", type);
                throw std::runtime_error("Unsupported shape type: " + type);
            }
            continue;
        }

        const auto filename_node = detail::find_named_child(shape_node, "string", "filename");
        if (!filename_node) {
            log->error("PBPT XML import failed: obj shape missing filename property.");
            throw std::runtime_error("obj shape is missing filename property.");
        }

        std::filesystem::path mesh_path =
            detail::validate_and_normalize_mesh_path(filename_node.attribute("value").value());

        const auto ref_node = shape_node.child("ref");
        if (!ref_node) {
            log->error("PBPT XML import failed: shape missing material ref id.");
            throw std::runtime_error("shape is missing material ref id.");
        }

        const std::string bsdf_id = ref_node.attribute("id").value();
        if (!reflectance_by_bsdf_id.contains(bsdf_id)) {
            log->error("PBPT XML import failed: unknown shape material ref id '{}'.", bsdf_id);
            throw std::runtime_error("shape ref id is unknown: " + bsdf_id);
        }

        pbpt::math::mat4 model{1.0f};
        if (const auto transform_node = shape_node.child("transform"); transform_node) {
            model = detail::parse_shape_transform(transform_node, options.require_supported_cbox_subset);
        }

        auto& go = scene.create_game_object(
            detail::default_object_name(shape_node, mesh_path, shape_index)
        );
        detail::register_imported_game_object(result, go.name(), go.id());
        const auto mesh_rel_to_resource_root =
            (std::filesystem::path(location.scene_root_rel_to_resource_dir) / mesh_path).lexically_normal();
        const resource::MeshHandle mesh_handle =
            resources.create_from_relative_path<rtr::resource::MeshResourceKind>(mesh_rel_to_resource_root.generic_string());
        const component::PbptRgb base_rgb = reflectance_by_bsdf_id.at(bsdf_id);
        (void)go.add_component<component::MeshRenderer>(
            mesh_handle,
            pbpt::math::vec4{base_rgb.r, base_rgb.g, base_rgb.b, 1.0f}
        );
        (void)go.add_component<component::PbptMesh>();
        go.node().set_local_model_matrix(model);

        if (const auto emitter_node = shape_node.child("emitter"); emitter_node) {
            const std::string emitter_type = emitter_node.attribute("type").value();
            if (emitter_type != "area" && options.require_supported_cbox_subset) {
                log->error("PBPT XML import failed: unsupported emitter type '{}'.", emitter_type);
                throw std::runtime_error("Unsupported emitter type: " + emitter_type);
            }

            const auto radiance_node = detail::find_named_child(emitter_node, "spectrum", "radiance");
            if (!radiance_node) {
                log->error("PBPT XML import failed: area emitter missing radiance spectrum.");
                throw std::runtime_error("area emitter is missing radiance spectrum.");
            }

            const std::string value = radiance_node.attribute("value").value();
            if (value.empty()) {
                log->error("PBPT XML import failed: area emitter radiance spectrum is empty.");
                throw std::runtime_error("area emitter radiance spectrum is empty.");
            }

            auto& pbpt_light = go.add_component<component::PbptLight>();
            pbpt_light.set_radiance_spectrum(
                detail::parse_pbpt_spectrum(value, "shape.emitter.radiance")
            );
            ++result.imported_light_shape_count;
        }

        ++result.imported_shape_count;
        ++shape_index;
    }

    scene.scene_graph().update_world_transforms();
    log->info(
        "PBPT XML import completed (shapes={}, lights={}, camera_imported={}).",
        result.imported_shape_count,
        result.imported_light_shape_count,
        result.sensor.has_value()
    );
    return result;
}

} // namespace rtr::framework::integration
