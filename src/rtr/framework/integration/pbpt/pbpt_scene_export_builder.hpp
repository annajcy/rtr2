#pragma once

#include <pbpt/math/math.h>
#include <pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp>
#include "pbpt/camera/fov_axis.hpp"
#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/camera/plugin/film/hdr_film.hpp"
#include "pbpt/camera/plugin/pixel_filter/gaussian_filter.hpp"
#include "pbpt/camera/pixel_sensor.hpp"
#include "pbpt/geometry/transform.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "pbpt/serde/scene_writer.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "rtr/framework/core/camera.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/pbpt/bridge/context.hpp"
#include "rtr/framework/integration/pbpt/bridge/dispatch.hpp"
#include "rtr/framework/integration/pbpt/bridge/export_helpers.hpp"
#include "rtr/framework/integration/pbpt/bridge/registry.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::framework::integration {

inline std::shared_ptr<spdlog::logger> pbpt_export_logger() {
    return utils::get_logger("framework.integration.pbpt.export");
}

namespace detail {
namespace compat_detail {

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
        compat_export_detail::to_transform(active_camera->node().world_matrix()), pbpt::camera::RenderSpace::World);

    PbptCompatibleInfo                           empty_compatible_info{};
    std::unordered_map<std::string, std::string> material_name_by_reflectance{};
    ExportGlobalContext                          ctx{.scene     = scene,
                                                     .resources = resources,
                                                     .compatible_info = compatible_info != nullptr ? *compatible_info : empty_compatible_info,
                                                     .material_name_by_reflectance = material_name_by_reflectance};

    for (const auto node_id : scene.scene_graph().active_nodes()) {
        const auto* go = scene.find_game_object(node_id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }

        const auto* pbpt_light = go->get_component<component::PbptLight>();
        const auto* pbpt_mesh  = go->get_component<component::PbptMesh>();
        if (pbpt_light != nullptr && pbpt_mesh == nullptr) {
            throw std::runtime_error("PbptLight requires PbptMesh on the same GameObject for export.");
        }

        try {
            auto shape_res = dispatch_impl(ExportShapeMapperList{}, *go, ctx, result);
            if (shape_res.matched) {
                log->debug("Shape {} mapped by {}", go->name(), shape_res.mapper_name);
            }
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("[domain=shape] ") + e.what());
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
