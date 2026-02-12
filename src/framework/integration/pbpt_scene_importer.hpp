#pragma once

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
#include <vector>

#include <glm/ext/matrix_transform.hpp>
#include <glm/mat4x4.hpp>
#include <glm/vec3.hpp>
#include <pugixml.hpp>

#include "framework/component/mesh_renderer.hpp"
#include "framework/component/pbpt_light.hpp"
#include "framework/component/pbpt_mesh.hpp"
#include "framework/component/pbpt_spectrum.hpp"
#include "framework/core/camera.hpp"
#include "framework/core/scene.hpp"
#include "framework/integration/pbpt_scene_export_builder.hpp"

namespace rtr::framework::integration {

struct PbptImportOptions {
    bool resolve_mesh_to_absolute{true};
    bool require_supported_cbox_subset{true};
};

struct PbptImportResult {
    std::size_t imported_shape_count{0};
    std::size_t imported_light_shape_count{0};
    std::optional<PbptIntegratorRecord> integrator{};
    std::optional<PbptSensorRecord> sensor{};
};

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

inline glm::vec3 parse_vec3_csv(const std::string& text, std::string_view field_name) {
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

inline glm::mat4 parse_matrix_row_major(const std::string& text, std::string_view field_name) {
    const auto values = parse_float_list(text, field_name);
    if (values.size() != 16u) {
        throw std::runtime_error(std::string(field_name) + " must contain exactly 16 float values.");
    }

    glm::mat4 matrix{1.0f};
    std::size_t idx = 0;
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[col][row] = values[idx++];
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

inline glm::mat4 parse_shape_transform(
    const pugi::xml_node& transform_node,
    bool strict_mode
) {
    glm::mat4 transform{1.0f};

    for (const auto& child : transform_node.children()) {
        const std::string_view tag = child.name();
        if (tag == "matrix") {
            const std::string value = child.attribute("value").value();
            if (value.empty()) {
                throw std::runtime_error("shape transform matrix is missing value.");
            }
            transform *= parse_matrix_row_major(value, "shape.transform.matrix");
            continue;
        }

        if (tag == "translate") {
            const float x = child.attribute("x").as_float(0.0f);
            const float y = child.attribute("y").as_float(0.0f);
            const float z = child.attribute("z").as_float(0.0f);
            transform *= glm::translate(glm::mat4{1.0f}, glm::vec3{x, y, z});
            continue;
        }

        if (strict_mode) {
            throw std::runtime_error("Unsupported shape transform element: " + std::string(tag));
        }
    }

    return transform;
}

inline glm::mat4 parse_sensor_to_world(
    const pugi::xml_node& transform_node,
    bool strict_mode
) {
    bool has_look_at = false;
    bool has_matrix = false;
    glm::mat4 to_world{1.0f};

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

            const glm::vec3 origin = parse_vec3_csv(origin_text, "sensor.lookAt.origin");
            const glm::vec3 target = parse_vec3_csv(target_text, "sensor.lookAt.target");
            const glm::vec3 up = parse_vec3_csv(up_text, "sensor.lookAt.up");
            to_world = glm::inverse(glm::lookAt(origin, target, up));
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

} // namespace detail

inline PbptImportResult import_pbpt_scene_xml_to_scene(
    const std::string& xml_path,
    core::Scene& scene,
    const PbptImportOptions& options = {}
) {
    if (xml_path.empty()) {
        throw std::invalid_argument("xml_path must not be empty.");
    }

    pugi::xml_document doc;
    const pugi::xml_parse_result parse_result = doc.load_file(xml_path.c_str());
    if (!parse_result) {
        throw std::runtime_error(std::string("XML load error: ") + parse_result.description());
    }

    const pugi::xml_node root = doc.child("scene");
    if (!root) {
        throw std::runtime_error("XML root node <scene> is missing.");
    }

    PbptImportResult result{};
    std::unordered_map<std::string, component::PbptSpectrum> reflectance_by_bsdf_id{};

    const std::filesystem::path input_path = std::filesystem::absolute(std::filesystem::path(xml_path));
    const std::filesystem::path base_dir = input_path.parent_path();

    for (const auto& bsdf_node : root.children("bsdf")) {
        const std::string type = bsdf_node.attribute("type").value();
        const std::string id = bsdf_node.attribute("id").value();
        if (id.empty()) {
            throw std::runtime_error("bsdf id is required.");
        }

        if (type != "diffuse") {
            if (options.require_supported_cbox_subset) {
                throw std::runtime_error("Unsupported bsdf type: " + type);
            }
            continue;
        }

        const auto spectrum_node = detail::find_named_child(bsdf_node, "spectrum", "reflectance");
        if (!spectrum_node) {
            throw std::runtime_error("diffuse bsdf is missing reflectance spectrum.");
        }

        const std::string value = spectrum_node.attribute("value").value();
        if (value.empty()) {
            throw std::runtime_error("diffuse reflectance spectrum is empty.");
        }

        reflectance_by_bsdf_id[id] = detail::parse_pbpt_spectrum(value, "bsdf.reflectance");
    }

    if (const auto integrator_node = root.child("integrator"); integrator_node) {
        PbptIntegratorRecord integrator{};
        integrator.type = integrator_node.attribute("type").value();
        if (integrator.type.empty()) {
            throw std::runtime_error("integrator type is required.");
        }
        if (integrator.type != "path" && options.require_supported_cbox_subset) {
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

        result.sensor = sensor;
    }

    std::size_t shape_index = 0;
    for (const auto& shape_node : root.children("shape")) {
        const std::string type = shape_node.attribute("type").value();
        if (type != "obj") {
            if (options.require_supported_cbox_subset) {
                throw std::runtime_error("Unsupported shape type: " + type);
            }
            continue;
        }

        const auto filename_node = detail::find_named_child(shape_node, "string", "filename");
        if (!filename_node) {
            throw std::runtime_error("obj shape is missing filename property.");
        }

        std::filesystem::path mesh_path = filename_node.attribute("value").value();
        if (mesh_path.empty()) {
            throw std::runtime_error("obj filename must not be empty.");
        }
        if (options.resolve_mesh_to_absolute && mesh_path.is_relative()) {
            mesh_path = (base_dir / mesh_path).lexically_normal();
        }

        const auto ref_node = shape_node.child("ref");
        if (!ref_node) {
            throw std::runtime_error("shape is missing material ref id.");
        }

        const std::string bsdf_id = ref_node.attribute("id").value();
        if (!reflectance_by_bsdf_id.contains(bsdf_id)) {
            throw std::runtime_error("shape ref id is unknown: " + bsdf_id);
        }

        glm::mat4 model{1.0f};
        if (const auto transform_node = shape_node.child("transform"); transform_node) {
            model = detail::parse_shape_transform(transform_node, options.require_supported_cbox_subset);
        }

        auto& go = scene.create_game_object(
            detail::default_object_name(shape_node, mesh_path, shape_index)
        );
        (void)go.add_component<component::MeshRenderer>(mesh_path.string(), "");
        auto& pbpt_mesh = go.add_component<component::PbptMesh>();
        pbpt_mesh.set_reflectance_spectrum(reflectance_by_bsdf_id.at(bsdf_id));
        go.node().set_local_model_matrix(model);

        if (const auto emitter_node = shape_node.child("emitter"); emitter_node) {
            const std::string emitter_type = emitter_node.attribute("type").value();
            if (emitter_type != "area" && options.require_supported_cbox_subset) {
                throw std::runtime_error("Unsupported emitter type: " + emitter_type);
            }

            const auto radiance_node = detail::find_named_child(emitter_node, "spectrum", "radiance");
            if (!radiance_node) {
                throw std::runtime_error("area emitter is missing radiance spectrum.");
            }

            const std::string value = radiance_node.attribute("value").value();
            if (value.empty()) {
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
    return result;
}

} // namespace rtr::framework::integration
