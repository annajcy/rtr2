#pragma once

#include <pbpt/math/math.h>
#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/light/plugin/light/area_light.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/serde/scene_writer.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"
#include "pbpt/shape/primitive.hpp"

#include <cstdint>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/pbpt_compatible_info.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/framework/integration/pbpt/pbpt_scene_metadata.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/log.hpp"
#include "rtr/utils/obj_io.hpp"

namespace rtr::framework::integration {

inline std::shared_ptr<spdlog::logger> pbpt_export_logger() {
    return utils::get_logger("framework.integration.pbpt.export");
}

namespace detail {

inline std::string rgb_value_string(const component::PbptRgb& rgb) {
    component::validate_pbpt_rgb(rgb, "PbptShapeRecord.reflectance_rgb");
    std::ostringstream oss;
    oss << std::setprecision(6) << rgb.r << " " << rgb.g << " " << rgb.b;
    return oss.str();
}

inline std::string reflectance_key(const component::PbptRgb& reflectance) {
    return "rgb:" + rgb_value_string(reflectance);
}

namespace compat_detail {

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

inline pbpt::geometry::Transform<float> to_transform(const pbpt::math::mat4& matrix) {
    return pbpt::geometry::Transform<float>(matrix);
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

inline pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float> to_piecewise_spectrum(
    const component::PbptSpectrum& spectrum) {
    component::validate_pbpt_spectrum(spectrum, "pbpt_light.radiance_spectrum");
    std::vector<std::pair<float, float>> points{};
    points.reserve(spectrum.size());
    for (const auto& point : spectrum) {
        points.emplace_back(point.lambda_nm, point.value);
    }
    return pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>(points);
}

inline pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float> rgb_to_piecewise(const component::PbptRgb& rgb) {
    const auto spectrum = pbpt_rgb_to_spectrum(rgb);
    return to_piecewise_spectrum(spectrum);
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

template <typename ExistsFn>
std::string make_unique_name(std::string base, ExistsFn&& exists) {
    if (!exists(base)) {
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!exists(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("Failed to generate unique resource name.");
}

inline std::string make_unique_shape_id(std::string base, std::unordered_set<std::string>& used_shape_ids) {
    if (base.empty()) {
        base = "shape";
    }
    if (!used_shape_ids.contains(base)) {
        used_shape_ids.insert(base);
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!used_shape_ids.contains(candidate)) {
            used_shape_ids.insert(candidate);
            return candidate;
        }
    }
    throw std::runtime_error("Failed to generate unique shape id.");
}

inline pbpt::shape::TriangleMesh<float> to_pbpt_triangle_mesh(
    const utils::ObjMeshData& mesh, const pbpt::camera::RenderTransform<float>& render_transform,
    const pbpt::geometry::Transform<float>& object_to_world) {
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::runtime_error("Cannot convert empty RTR mesh to PBPT mesh.");
    }

    std::vector<int> indices{};
    indices.reserve(mesh.indices.size());
    for (const auto idx : mesh.indices) {
        if (idx > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("RTR mesh index exceeds int range for PBPT conversion.");
        }
        indices.emplace_back(static_cast<int>(idx));
    }

    std::vector<pbpt::math::Point<float, 3>>  positions{};
    std::vector<pbpt::math::Normal<float, 3>> normals{};
    std::vector<pbpt::math::Point<float, 2>>  uvs{};
    positions.reserve(mesh.vertices.size());
    normals.reserve(mesh.vertices.size());
    uvs.reserve(mesh.vertices.size());
    for (const auto& vertex : mesh.vertices) {
        positions.emplace_back(vertex.position.x(), vertex.position.y(), vertex.position.z());
        normals.emplace_back(vertex.normal.x(), vertex.normal.y(), vertex.normal.z());
        uvs.emplace_back(vertex.uv.x(), vertex.uv.y());
    }

    return pbpt::shape::TriangleMesh<float>(render_transform, indices, positions, normals, uvs, false, object_to_world);
}

inline std::vector<pbpt::shape::Primitive<float>> build_primitives_from_resources(
    const pbpt::scene::RenderResources<float>& resources) {
    std::vector<pbpt::shape::Primitive<float>> primitives{};
    for (const auto& record : resources.shape_instances) {
        if (!resources.mesh_library.name_to_id().contains(record.mesh_name)) {
            throw std::runtime_error("PBPT export merge failed: shape references missing mesh '" + record.mesh_name +
                                     "'.");
        }
        const auto& mesh = resources.mesh_library.get(record.mesh_name);

        if (!resources.mesh_material_map.contains(record.mesh_name)) {
            throw std::runtime_error("PBPT export merge failed: mesh has no material assignment '" + record.mesh_name +
                                     "'.");
        }
        const int material_id = resources.mesh_material_map.at(record.mesh_name);
        if (!resources.any_material_library.id_to_name().contains(material_id)) {
            throw std::runtime_error("PBPT export merge failed: material id is unknown for mesh '" + record.mesh_name +
                                     "'.");
        }

        for (int triangle_index = 0; triangle_index < mesh.triangle_count(); ++triangle_index) {
            int        light_id = -1;
            const auto key      = pbpt::scene::make_mesh_triangle_key(record.mesh_name, triangle_index);
            if (resources.mesh_light_map.contains(key)) {
                light_id = resources.mesh_light_map.at(key);
                if (!resources.any_light_library.id_to_name().contains(light_id)) {
                    throw std::runtime_error("PBPT export merge failed: light id is unknown for mesh triangle.");
                }
            }
            primitives.emplace_back(pbpt::shape::Triangle<float>(mesh, triangle_index), material_id, light_id);
        }
    }
    return primitives;
}

inline pbpt::integrator::AnyIntegrator<float> make_default_integrator() {
    return pbpt::integrator::PathIntegrator<float, 4>(static_cast<unsigned>(-1), 0.9f);
}

inline pbpt::serde::PbptXmlResult<float> make_initial_xml_result(const PbptCompatibleInfo* compatible_info) {
    if (compatible_info == nullptr) {
        pbpt::serde::PbptXmlResult<float> result{};
        result.integrator = make_default_integrator();
        result.spp        = 4;
        return result;
    }

    return pbpt::serde::PbptXmlResult<float>{
        .integrator = compatible_info->passthrough_integrator.value_or(make_default_integrator()),
        .scene      = pbpt::scene::Scene<float>{.resources = compatible_info->passthrough_resources},
        .spp        = std::max(1, compatible_info->passthrough_spp)};
}

inline void apply_compatible_passthrough(const PbptCompatibleInfo&          compatible_info,
                                         pbpt::serde::PbptXmlResult<float>& result) {
    if (compatible_info.passthrough_shape_ids.empty()) {
        result.scene.resources.shape_instances.clear();
        return;
    }

    std::unordered_set<std::string> seen_shape_ids{};
    for (const auto& record : result.scene.resources.shape_instances) {
        seen_shape_ids.insert(record.shape_id);
    }
    for (const auto& expected_id : compatible_info.passthrough_shape_ids) {
        if (!seen_shape_ids.contains(expected_id)) {
            throw std::runtime_error("compatible_info passthrough shape id not found in passthrough resources: " +
                                     expected_id);
        }
    }

    std::erase_if(result.scene.resources.shape_instances, [&compatible_info](const auto& record) {
        return !compatible_info.passthrough_shape_ids.contains(record.shape_id);
    });

    std::unordered_set<std::string> mapped_source_shape_ids{};
    for (const auto& [_, mapped] : compatible_info.mapped_shape_info_by_game_object) {
        mapped_source_shape_ids.insert(mapped.source_shape_id);
    }
    for (const auto& passthrough_id : compatible_info.passthrough_shape_ids) {
        if (mapped_source_shape_ids.contains(passthrough_id)) {
            throw std::runtime_error("compatible_info violation: mapped shape id also exists in passthrough set: " +
                                     passthrough_id);
        }
    }
}

inline pbpt::camera::AnyCamera<float> build_pbpt_camera(const core::PerspectiveCamera& camera, int film_width,
                                                        int film_height) {
    const int width  = std::max(film_width, 1);
    const int height = std::max(film_height, 1);

    auto pixel_sensor = pbpt::camera::PixelSensor<float, pbpt::radiometry::constant::CIED65SpectrumType<float>,
                                                  pbpt::radiometry::constant::CIED65SpectrumType<float>,
                                                  pbpt::radiometry::constant::XYZSpectrumType<float>>(
        pbpt::radiometry::constant::CIE_D65_ilum<float>, pbpt::radiometry::constant::CIE_D65_ilum<float>,
        pbpt::radiometry::constant::sRGB<float>,
        pbpt::radiometry::ResponseSpectrum<pbpt::radiometry::constant::XYZSpectrumType<float>>(
            pbpt::radiometry::constant::CIE_X<float>, pbpt::radiometry::constant::CIE_Y<float>,
            pbpt::radiometry::constant::CIE_Z<float>),
        1.0f);
    auto film =
        pbpt::camera::HDRFilm<float, decltype(pixel_sensor)>(pbpt::math::Vector<int, 2>(width, height), pixel_sensor);

    return pbpt::camera::ThinLensPerspectiveCamera<float>(
        pbpt::camera::AnyFilm<float>(std::move(film)), camera.fov_degrees(), pbpt::camera::FovAxis::Smaller,
        -std::max(camera.near_bound(), 1e-4f), -std::max(camera.far_bound(), 1e-3f), 1000.0f);
}

}  // namespace compat_detail
}  // namespace detail

inline pbpt::serde::PbptXmlResult<float> build_pbpt_xml_result_from_scene(
    const core::Scene& scene, resource::ResourceManager& resources, const PbptCompatibleInfo* compatible_info = nullptr,
    int film_width_override = 0, int film_height_override = 0, int spp_override = 0) {
    auto                              log    = pbpt_export_logger();
    pbpt::serde::PbptXmlResult<float> result = detail::compat_detail::make_initial_xml_result(compatible_info);

    if (compatible_info != nullptr) {
        detail::compat_detail::apply_compatible_passthrough(*compatible_info, result);
    }

    if (spp_override > 0) {
        result.spp = spp_override;
    }

    const auto* active_camera = dynamic_cast<const core::PerspectiveCamera*>(scene.active_camera());
    if (active_camera == nullptr) {
        throw std::runtime_error("PBPT export requires an active perspective camera.");
    }

    const int film_width          = film_width_override > 0 ? film_width_override : 512;
    const int film_height         = film_height_override > 0 ? film_height_override : 512;
    result.scene.camera           = detail::compat_detail::build_pbpt_camera(*active_camera, film_width, film_height);
    result.scene.pixel_filter     = pbpt::camera::GaussianFilter<float>(1.5f, 0.5f);
    result.scene.render_transform = pbpt::camera::RenderTransform<float>::from_camera_to_world(
        detail::compat_detail::to_transform(active_camera->node().world_matrix()), pbpt::camera::RenderSpace::World);

    std::unordered_set<std::string> used_shape_ids{};
    for (const auto& shape_record : result.scene.resources.shape_instances) {
        used_shape_ids.insert(shape_record.shape_id);
    }

    std::unordered_map<std::string, std::string> material_name_by_reflectance{};

    for (const auto node_id : scene.scene_graph().active_nodes()) {
        const auto* go = scene.find_game_object(node_id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }

        const auto* mesh_renderer = go->get_component<component::MeshRenderer>();
        const auto* pbpt_mesh     = go->get_component<component::PbptMesh>();
        const auto* pbpt_light    = go->get_component<component::PbptLight>();
        if (pbpt_light != nullptr && pbpt_mesh == nullptr) {
            throw std::runtime_error("PbptLight requires PbptMesh on the same GameObject for export.");
        }
        if (mesh_renderer == nullptr || pbpt_mesh == nullptr) {
            continue;
        }
        if (!mesh_renderer->enabled() || !pbpt_mesh->enabled()) {
            continue;
        }

        const auto mesh_handle = pbpt_mesh->mesh_handle();
        if (!mesh_handle.is_valid() || !resources.alive<rtr::resource::MeshResourceKind>(mesh_handle)) {
            throw std::runtime_error("Pbpt export requires valid and alive mesh handle.");
        }
        const auto& cpu_mesh        = resources.cpu<rtr::resource::MeshResourceKind>(mesh_handle);
        const auto  object_to_world = detail::compat_detail::to_transform(scene.scene_graph().node(node_id).world_matrix());

        std::string mesh_name = detail::compat_detail::make_unique_name(
            "rtr_mesh_" + std::to_string(mesh_handle.value),
            [&](const std::string& name) { return result.scene.resources.mesh_library.name_to_id().contains(name); });
        auto mesh = detail::compat_detail::to_pbpt_triangle_mesh(cpu_mesh, result.scene.render_transform, object_to_world);
        (void)result.scene.resources.mesh_library.add_item(mesh_name, std::move(mesh));

        const pbpt::math::vec4   base_color = mesh_renderer->base_color();
        const component::PbptRgb reflectance{.r = base_color.x(), .g = base_color.y(), .b = base_color.z()};
        component::validate_pbpt_rgb(reflectance, "MeshRenderer.base_color");
        const std::string reflectance_key = detail::reflectance_key(reflectance);

        std::string material_name{};
        if (material_name_by_reflectance.contains(reflectance_key)) {
            material_name = material_name_by_reflectance.at(reflectance_key);
        } else {
            material_name = detail::compat_detail::make_unique_name(
                "rtr_mat_" + std::to_string(material_name_by_reflectance.size()), [&](const std::string& name) {
                    return result.scene.resources.any_material_library.name_to_id().contains(name);
                });
            auto material = pbpt::material::LambertianMaterial<float>(detail::compat_detail::rgb_to_piecewise(reflectance));
            (void)result.scene.resources.any_material_library.add_item(material_name, std::move(material));
            material_name_by_reflectance.emplace(reflectance_key, material_name);
        }
        result.scene.resources.mesh_material_map[mesh_name] =
            result.scene.resources.any_material_library.name_to_id().at(material_name);

        std::string shape_base = go->name();
        if (shape_base.empty()) {
            shape_base = "go_" + std::to_string(static_cast<std::uint64_t>(go->id()));
        }
        const std::string shape_id = detail::compat_detail::make_unique_shape_id(shape_base, used_shape_ids);

        pbpt::scene::ShapeInstanceRecord<float> shape_record{};
        shape_record.shape_id          = shape_id;
        shape_record.shape_type        = "obj";
        shape_record.mesh_name         = mesh_name;
        shape_record.material_ref_name = material_name;
        shape_record.object_to_world   = object_to_world;

        if (pbpt_light != nullptr && pbpt_light->enabled()) {
            const std::string emission_name =
                detail::compat_detail::make_unique_name(shape_id + "_emission", [&](const std::string& name) {
                    return result.scene.resources.reflectance_spectrum_library.name_to_id().contains(name);
                });
            result.scene.resources.reflectance_spectrum_library.add_item(
                emission_name, detail::compat_detail::to_piecewise_spectrum(pbpt_light->area_emitter().radiance_spectrum));
            shape_record.emission_spectrum_name = emission_name;
        }

        result.scene.resources.shape_instances.emplace_back(shape_record);

        if (shape_record.emission_spectrum_name.has_value()) {
            const auto& mesh_ref = result.scene.resources.mesh_library.get(mesh_name);
            const auto& emission_spectrum =
                result.scene.resources.reflectance_spectrum_library.get(shape_record.emission_spectrum_name.value());
            auto light_spectrum = pbpt::radiometry::StandardEmissionSpectrum<float>(
                emission_spectrum, pbpt::radiometry::constant::CIE_D65_ilum<float>);
            for (int triangle_index = 0; triangle_index < mesh_ref.triangle_count(); ++triangle_index) {
                const std::string light_name = detail::compat_detail::make_unique_name(
                    shape_id + "_light_" + std::to_string(triangle_index), [&](const std::string& name) {
                        return result.scene.resources.any_light_library.name_to_id().contains(name);
                    });
                const int light_id = result.scene.resources.any_light_library.add_item(
                    light_name, pbpt::light::AreaLight<float, pbpt::shape::Triangle<float>, decltype(light_spectrum)>(
                                    pbpt::shape::Triangle<float>(mesh_ref, triangle_index), light_spectrum,
                                    pbpt::light::AreaLightSamplingDomain::Shape));
                result.scene.resources.mesh_light_map[pbpt::scene::make_mesh_triangle_key(mesh_name, triangle_index)] =
                    light_id;
            }
        }
    }

    const auto primitives  = detail::compat_detail::build_primitives_from_resources(result.scene.resources);
    result.scene.aggregate = pbpt::aggregate::EmbreeAggregate<float>(primitives);

    log->debug("Built PBPT XmlResult from RTR scene (shape_count={}, passthrough_shape_count={}, spp={}).",
               result.scene.resources.shape_instances.size(),
               compatible_info != nullptr ? compatible_info->passthrough_shape_ids.size() : 0u, result.spp);
    return result;
}

inline void write_pbpt_xml_result_to_path(const pbpt::serde::PbptXmlResult<float>& result,
                                          const std::string&                       scene_xml_path) {
    if (scene_xml_path.empty()) {
        throw std::invalid_argument("scene_xml_path must not be empty.");
    }
    pbpt::serde::write_scene<float>(result, scene_xml_path);
}

}  // namespace rtr::framework::integration
