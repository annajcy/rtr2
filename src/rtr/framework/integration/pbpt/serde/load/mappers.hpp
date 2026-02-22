#pragma once

#include <string_view>
#include <algorithm>
#include <limits>
#include <stdexcept>
#include <variant>

#include "rtr/framework/integration/pbpt/serde/domain/trait_contracts.hpp"
#include "rtr/framework/integration/pbpt/serde/load/types.hpp"
#include "rtr/framework/integration/pbpt/serde/load/helpers.hpp"

#include "pbpt/camera/plugin/camera/projective_cameras.hpp"
#include "pbpt/integrator/plugin/integrator/path_integrator.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/framework/core/scene.hpp"

namespace rtr::framework::integration {

struct ObjLambertianShapeImportMapper {
    static constexpr std::string_view kName = "ObjLambertianShapeImportMapper";

    static bool matches(const ::pbpt::scene::ShapeInstanceRecord<float>& record, const ImportGlobalContext& ctx,
                        LoadPackage&) {
        if (record.shape_type != "obj")
            return false;

        const auto& resources = ctx.pbpt_scene_result.scene.resources;
        if (!resources.mesh_library.name_to_id().contains(record.mesh_name))
            return false;
        if (!resources.any_material_library.name_to_id().contains(record.material_ref_name))
            return false;

        const int material_id = resources.any_material_library.name_to_id().at(record.material_ref_name);
        if (!resources.any_material_library.id_to_name().contains(material_id))
            return false;

        const auto& any_material = resources.any_material_library.get(material_id);
        if (!std::holds_alternative<::pbpt::material::LambertianMaterial<float>>(any_material))
            return false;

        if (record.emission_spectrum_name.has_value() &&
            !resources.reflectance_spectrum_library.name_to_id().contains(record.emission_spectrum_name.value())) {
            return false;
        }
        return true;
    }

    static void map(const ::pbpt::scene::ShapeInstanceRecord<float>& record, const ImportGlobalContext& ctx,
                    LoadPackage& pkg) {
        const auto& resources    = ctx.pbpt_scene_result.scene.resources;
        const auto& mesh         = resources.mesh_library.get(record.mesh_name);
        const auto  material_id  = resources.any_material_library.name_to_id().at(record.material_ref_name);
        const auto& any_material = resources.any_material_library.get(material_id);
        const auto* lambertian   = std::get_if<::pbpt::material::LambertianMaterial<float>>(&any_material);

        component::PbptRgb reflectance{};
        try {
            reflectance = compat_import_detail::lambertian_to_rgb(*lambertian);
        } catch (const std::exception&) {
            pkg.compatible_info.passthrough_shape_ids.insert(record.shape_id);
            return;
        }

        auto       cpu_mesh    = compat_import_detail::to_rtr_mesh_data(mesh);
        const auto mesh_handle = ctx.resources.create<rtr::resource::MeshResourceKind>(std::move(cpu_mesh));

        std::string object_name = record.shape_id.empty() ? record.mesh_name : record.shape_id;
        if (object_name.empty())
            object_name = "shape";

        object_name = compat_import_detail::make_unique_name(object_name, pkg.result.imported_game_object_id_by_name);

        auto& go = ctx.scene.create_game_object(object_name);
        (void)go.add_component<component::MeshRenderer>(
            mesh_handle, ::pbpt::math::vec4{reflectance.r, reflectance.g, reflectance.b, 1.0f});
        (void)go.add_component<component::PbptMesh>();
        go.node().set_local_model_matrix(compat_import_detail::to_mat4(record.object_to_world));

        if (record.emission_spectrum_name.has_value()) {
            const auto& emission = resources.reflectance_spectrum_library.get(record.emission_spectrum_name.value());
            auto        preview  = compat_import_detail::area_emission_to_preview_point_light(emission);
            auto        spectrum = compat_import_detail::to_component_spectrum(emission);

            auto&       light    = go.add_component<component::PbptLight>();
            light.set_radiance_spectrum(std::move(spectrum));

            auto& point_light = go.add_component<component::light::PointLight>();
            point_light.set_color(preview.color);
            point_light.set_intensity(preview.intensity);
            ++pkg.result.imported_light_shape_count;
        }

        compat_import_detail::register_imported_game_object(pkg.result, go.name(), go.id());
        pkg.compatible_info.mapped_shape_info_by_game_object.emplace(
            go.id(), MappedShapeInfo{.source_shape_id          = record.shape_id,
                                     .source_mesh_name         = record.mesh_name,
                                     .source_material_ref_name = record.material_ref_name});
        ++pkg.result.imported_shape_count;
    }
};

struct ThinLensPerspectiveImportMapper {
    static constexpr std::string_view kName = "ThinLensPerspectiveImportMapper";

    static bool matches(const ::pbpt::camera::AnyCamera<float>& camera_any, const ImportGlobalContext&,
                        LoadPackage&) {
        return std::holds_alternative<::pbpt::camera::ThinLensPerspectiveCamera<float>>(camera_any);
    }

    static void map(const ::pbpt::camera::AnyCamera<float>& camera_any, const ImportGlobalContext& ctx,
                    LoadPackage& pkg) {
        const auto& camera_pbpt = std::get<::pbpt::camera::ThinLensPerspectiveCamera<float>>(camera_any);

        SensorRecord sensor{};
        sensor.to_world = compat_import_detail::to_mat4(ctx.pbpt_scene_result.scene.render_transform.camera_to_world());
        sensor.fov_degrees    = camera_pbpt.fov_degrees();
        sensor.fov_axis       = ::pbpt::camera::fov_axis_to_string(camera_pbpt.fov_axis());
        sensor.near_clip      = -camera_pbpt.near_clip();
        sensor.far_clip       = -camera_pbpt.far_clip();
        sensor.focus_distance = camera_pbpt.focal_distance();
        sensor.film_width     = camera_pbpt.width();
        sensor.film_height    = camera_pbpt.height();
        sensor.sample_count   = std::max(1, ctx.pbpt_scene_result.spp);
        pkg.result.sensor     = sensor;

        std::string camera_name =
            compat_import_detail::make_unique_name("pbpt_camera", pkg.result.imported_game_object_id_by_name);
        auto& camera_go      = ctx.scene.create_game_object(camera_name);
        auto& camera         = ctx.scene.camera_manager().create_perspective_camera(camera_go.id());
        camera.near_bound()  = std::max(sensor.near_clip, 1e-4f);
        camera.far_bound()   = std::max(sensor.far_clip, camera.near_bound() + 1e-3f);
        camera.fov_degrees() = sensor.fov_degrees;
        camera.set_aspect_ratio(static_cast<float>(sensor.film_width) /
                                static_cast<float>(std::max(1, sensor.film_height)));
        camera_go.node().set_local_model_matrix(sensor.to_world);
        (void)ctx.scene.set_active_camera(camera_go.id());
        compat_import_detail::register_imported_game_object(pkg.result, camera_go.name(), camera_go.id());

        if (ctx.options.free_look_input_state != nullptr) {
            (void)camera_go.add_component<component::FreeLookCameraController>(ctx.options.free_look_input_state,
                                                                               &ctx.scene.camera_manager());
        }
    }
};

struct PathIntegratorImportMapper {
    static constexpr std::string_view kName = "PathIntegratorImportMapper";

    static bool matches(const ::pbpt::integrator::AnyIntegrator<float>& integrator, const ImportGlobalContext&,
                        LoadPackage&) {
        return std::holds_alternative<::pbpt::integrator::PathIntegrator<float, 4>>(integrator);
    }

    static void map(const ::pbpt::integrator::AnyIntegrator<float>& integrator, const ImportGlobalContext&,
                    LoadPackage& pkg) {
        const auto& path_integrator = std::get<::pbpt::integrator::PathIntegrator<float, 4>>(integrator);
        int         max_depth       = static_cast<int>(path_integrator.max_depth());
        if (path_integrator.max_depth() == std::numeric_limits<unsigned>::max()) {
            max_depth = -1;
        }
        pkg.result.integrator = IntegratorRecord{.type = "path", .max_depth = max_depth};
    }
};

}  // namespace rtr::framework::integration
