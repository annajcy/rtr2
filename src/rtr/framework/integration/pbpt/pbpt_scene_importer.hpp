#pragma once

#include <pbpt/math/math.h>
#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/serde/scene_loader.hpp"

#include <algorithm>
#include <cctype>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_compatible_info.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_metadata.hpp"
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
    std::size_t                                         imported_shape_count{0};
    std::size_t                                         imported_light_shape_count{0};
    std::unordered_map<std::string, core::GameObjectId> imported_game_object_id_by_name{};
    std::optional<PbptIntegratorRecord>                 integrator{};
    std::optional<PbptSensorRecord>                     sensor{};
};

struct PbptImportPackage {
    PbptImportResult   result{};
    PbptCompatibleInfo compatible_info{};
};

inline void register_imported_game_object(PbptImportResult& result, const std::string& name, core::GameObjectId id) {
    auto [_, inserted] = result.imported_game_object_id_by_name.emplace(name, id);
    if (!inserted) {
        throw std::runtime_error("Duplicate imported game object name: " + name);
    }
}

namespace compat_import_detail {

inline pbpt::math::mat4 to_mat4(const pbpt::geometry::Transform<float>& transform) {
    pbpt::math::mat4 matrix{1.0f};
    const auto&      src = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[row][col] = src.at(row, col);
        }
    }
    return matrix;
}

inline component::PbptSpectrum to_component_spectrum(
    const pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>& spectrum) {
    component::PbptSpectrum out{};
    out.reserve(spectrum.points().size());
    for (const auto& [lambda, value] : spectrum.points()) {
        out.emplace_back(component::PbptSpectrumPoint{.lambda_nm = lambda, .value = value});
    }
    component::validate_pbpt_spectrum(out, "piecewise_spectrum");
    return out;
}

inline component::PbptRgb lambertian_to_rgb(const pbpt::material::LambertianMaterial<float>& material) {
    const auto& source = material.reflectance_source();
    return std::visit(
        [](const auto& value) -> component::PbptRgb {
            using ValueT = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<ValueT, pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>>) {
                return pbpt_spectrum_to_rgb(to_component_spectrum(value));
            } else {
                throw std::runtime_error("Lambertian texture reflectance is not expressible by RTR MeshRenderer.");
            }
        },
        source);
}

inline utils::ObjMeshData to_rtr_mesh_data(const pbpt::shape::TriangleMesh<float>& mesh) {
    utils::ObjMeshData out{};
    const auto         render_to_object = mesh.render_to_object_transform();

    const auto& positions = mesh.positions();
    const auto& uvs       = mesh.uvs();
    const auto& normals   = mesh.normals();
    out.vertices.resize(positions.size());
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto object_p      = render_to_object.transform_point(positions[i]);
        out.vertices[i].position = pbpt::math::vec3(object_p.x(), object_p.y(), object_p.z());

        if (mesh.has_uvs() && i < uvs.size()) {
            out.vertices[i].uv = pbpt::math::vec2(uvs[i].x(), uvs[i].y());
        } else {
            out.vertices[i].uv = pbpt::math::vec2(0.0f, 0.0f);
        }

        if (mesh.has_normals() && i < normals.size()) {
            const auto object_n    = render_to_object.transform_normal(normals[i]).normalized();
            out.vertices[i].normal = pbpt::math::vec3(object_n.x(), object_n.y(), object_n.z());
        } else {
            out.vertices[i].normal = pbpt::math::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    out.indices.reserve(mesh.indices().size());
    for (const int idx : mesh.indices()) {
        if (idx < 0) {
            throw std::runtime_error("PBPT mesh contains negative index.");
        }
        out.indices.emplace_back(static_cast<std::uint32_t>(idx));
    }
    if (out.vertices.empty() || out.indices.empty()) {
        throw std::runtime_error("PBPT mesh conversion produced empty ObjMeshData.");
    }
    return out;
}

inline std::string make_unique_name(const std::string&                                         base,
                                    const std::unordered_map<std::string, core::GameObjectId>& existing) {
    if (!existing.contains(base)) {
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!existing.contains(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("Failed to create unique imported GameObject name.");
}

inline bool can_map_shape_to_rtr(const pbpt::scene::ShapeInstanceRecord<float>& shape,
                                 const pbpt::scene::RenderResources<float>&     resources) {
    if (shape.shape_type != "obj") {
        return false;
    }
    if (!resources.mesh_library.name_to_id().contains(shape.mesh_name)) {
        return false;
    }
    if (!resources.any_material_library.name_to_id().contains(shape.material_ref_name)) {
        return false;
    }

    const int material_id = resources.any_material_library.name_to_id().at(shape.material_ref_name);
    if (!resources.any_material_library.id_to_name().contains(material_id)) {
        return false;
    }
    const auto& any_material = resources.any_material_library.get(material_id);
    if (!std::holds_alternative<pbpt::material::LambertianMaterial<float>>(any_material)) {
        return false;
    }

    if (shape.emission_spectrum_name.has_value() &&
        !resources.reflectance_spectrum_library.name_to_id().contains(shape.emission_spectrum_name.value())) {
        return false;
    }
    return true;
}

}  // namespace compat_import_detail

inline PbptImportPackage import_pbpt_scene_to_scene_with_compatible(
    const pbpt::serde::PbptXmlResult<float>& pbpt_scene_result, core::Scene& scene,
    resource::ResourceManager& resources, const PbptImportOptions& options = {}) {
    (void)options;
    auto              log = pbpt_import_logger();
    PbptImportPackage package{
        .result          = PbptImportResult{},
        .compatible_info = PbptCompatibleInfo{
            .mapped_shape_info_by_game_object = {},
            .passthrough_resources            = pbpt_scene_result.scene.resources,
            .passthrough_shape_ids            = {},
            .passthrough_integrator = std::make_optional(pbpt_scene_result.integrator),
            .passthrough_spp = std::max(1, pbpt_scene_result.spp)}};

    std::visit(
        [&](const auto& integrator) {
            using IntegratorT = std::decay_t<decltype(integrator)>;
            if constexpr (std::is_same_v<IntegratorT, pbpt::integrator::PathIntegrator<float, 4>>) {
                int max_depth = static_cast<int>(integrator.max_depth());
                if (integrator.max_depth() == std::numeric_limits<unsigned>::max()) {
                    max_depth = -1;
                }
                package.result.integrator = PbptIntegratorRecord{.type = "path", .max_depth = max_depth};
            } else if (options.require_supported_cbox_subset) {
                throw std::runtime_error("Unsupported PBPT integrator in import.");
            }
        },
        pbpt_scene_result.integrator);

    std::visit(
        [&](const auto& camera_any) {
            using CameraT = std::decay_t<decltype(camera_any)>;
            if constexpr (std::is_same_v<CameraT, pbpt::camera::ThinLensPerspectiveCamera<float>>) {
                PbptSensorRecord sensor{};
                sensor.to_world = compat_import_detail::to_mat4(pbpt_scene_result.scene.render_transform.camera_to_world());
                sensor.fov_degrees    = camera_any.fov_degrees();
                sensor.fov_axis       = pbpt::camera::fov_axis_to_string(camera_any.fov_axis());
                sensor.near_clip      = -camera_any.near_clip();
                sensor.far_clip       = -camera_any.far_clip();
                sensor.focus_distance = camera_any.focal_distance();
                sensor.film_width     = camera_any.width();
                sensor.film_height    = camera_any.height();
                sensor.sample_count   = std::max(1, pbpt_scene_result.spp);
                package.result.sensor = sensor;

                std::string camera_name =
                    compat_import_detail::make_unique_name("pbpt_camera", package.result.imported_game_object_id_by_name);
                auto& camera_go      = scene.create_game_object(camera_name);
                auto& camera         = scene.camera_manager().create_perspective_camera(camera_go.id());
                camera.near_bound()  = std::max(sensor.near_clip, 1e-4f);
                camera.far_bound()   = std::max(sensor.far_clip, camera.near_bound() + 1e-3f);
                camera.fov_degrees() = sensor.fov_degrees;
                camera.set_aspect_ratio(static_cast<float>(sensor.film_width) /
                                        static_cast<float>(std::max(1, sensor.film_height)));
                camera_go.node().set_local_model_matrix(sensor.to_world);
                (void)scene.set_active_camera(camera_go.id());
                register_imported_game_object(package.result, camera_go.name(), camera_go.id());
                if (options.free_look_input_state != nullptr) {
                    (void)camera_go.add_component<component::FreeLookCameraController>(
                        options.free_look_input_state, &scene.camera_manager());
                }
            } else if (options.require_supported_cbox_subset) {
                throw std::runtime_error("Unsupported PBPT camera in import.");
            }
        },
        pbpt_scene_result.scene.camera);

    for (const auto& shape : pbpt_scene_result.scene.resources.shape_instances) {
        if (!compat_import_detail::can_map_shape_to_rtr(shape, pbpt_scene_result.scene.resources)) {
            package.compatible_info.passthrough_shape_ids.insert(shape.shape_id);
            continue;
        }

        const auto& mesh = pbpt_scene_result.scene.resources.mesh_library.get(shape.mesh_name);
        const auto  material_id =
            pbpt_scene_result.scene.resources.any_material_library.name_to_id().at(shape.material_ref_name);
        const auto& any_material = pbpt_scene_result.scene.resources.any_material_library.get(material_id);
        const auto* lambertian   = std::get_if<pbpt::material::LambertianMaterial<float>>(&any_material);
        if (lambertian == nullptr) {
            package.compatible_info.passthrough_shape_ids.insert(shape.shape_id);
            continue;
        }

        component::PbptRgb reflectance{};
        try {
            reflectance = compat_import_detail::lambertian_to_rgb(*lambertian);
        } catch (const std::exception&) {
            package.compatible_info.passthrough_shape_ids.insert(shape.shape_id);
            continue;
        }

        auto       cpu_mesh    = compat_import_detail::to_rtr_mesh_data(mesh);
        const auto mesh_handle = resources.create<rtr::resource::MeshResourceKind>(std::move(cpu_mesh));

        std::string object_name = shape.shape_id.empty() ? shape.mesh_name : shape.shape_id;
        if (object_name.empty()) {
            object_name = "shape";
        }
        object_name =
            compat_import_detail::make_unique_name(object_name, package.result.imported_game_object_id_by_name);

        auto& go = scene.create_game_object(object_name);
        (void)go.add_component<component::MeshRenderer>(
            mesh_handle, pbpt::math::vec4{reflectance.r, reflectance.g, reflectance.b, 1.0f});
        (void)go.add_component<component::PbptMesh>();
        go.node().set_local_model_matrix(compat_import_detail::to_mat4(shape.object_to_world));

        if (shape.emission_spectrum_name.has_value()) {
            const auto& emission = pbpt_scene_result.scene.resources.reflectance_spectrum_library.get(
                shape.emission_spectrum_name.value());
            auto& light = go.add_component<component::PbptLight>();
            light.set_radiance_spectrum(compat_import_detail::to_component_spectrum(emission));
            ++package.result.imported_light_shape_count;
        }

        register_imported_game_object(package.result, go.name(), go.id());
        package.compatible_info.mapped_shape_info_by_game_object.emplace(
            go.id(), PbptMappedShapeInfo{.source_shape_id          = shape.shape_id,
                                         .source_mesh_name         = shape.mesh_name,
                                         .source_material_ref_name = shape.material_ref_name});
        ++package.result.imported_shape_count;
    }

    scene.scene_graph().update_world_transforms();
    log->info("PBPT import with compatible info completed (mapped_shapes={}, mapped_lights={}, passthrough_shapes={}).",
              package.result.imported_shape_count, package.result.imported_light_shape_count,
              package.compatible_info.passthrough_shape_ids.size());
    return package;
}
inline PbptImportPackage import_pbpt_scene_xml_to_scene_with_compatible(const std::string&         scene_xml_path,
                                                                        core::Scene&               scene,
                                                                        resource::ResourceManager& resources,
                                                                        const PbptImportOptions&   options = {}) {
    if (scene_xml_path.empty()) {
        throw std::invalid_argument("scene_xml_path must not be empty.");
    }
    auto pbpt_scene_result = pbpt::serde::load_scene<float>(scene_xml_path);
    return import_pbpt_scene_to_scene_with_compatible(pbpt_scene_result, scene, resources, options);
}

inline PbptImportResult import_pbpt_scene_xml_to_scene(const std::string& scene_xml_path, core::Scene& scene,
                                                       resource::ResourceManager& resources,
                                                       const PbptImportOptions&   options = {}) {
    try {
        auto package = import_pbpt_scene_xml_to_scene_with_compatible(scene_xml_path, scene, resources, options);
        return package.result;
    } catch (const std::exception& e) {
        pbpt_import_logger()->error("PBPT XML import failed: {}", e.what());
        throw std::runtime_error(std::string("import_pbpt_scene_xml_to_scene failed: ") + e.what());
    }
}

}  // namespace rtr::framework::integration
