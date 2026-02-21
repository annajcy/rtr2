#pragma once

#include <string_view>
#include <algorithm>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>

#include "rtr/framework/integration/pbpt/bridge/trait_contracts.hpp"
#include "rtr/framework/integration/pbpt/bridge/export_helpers.hpp"

#include "pbpt/aggregate/plugin/aggregate/embree_aggregate.hpp"
#include "pbpt/light/plugin/light/area_light.hpp"
#include "pbpt/material/plugin/material/lambertian_material.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/pbpt/pbpt_light.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"

namespace rtr::framework::integration {

struct MeshRendererPbptMeshExportMapper {
    static constexpr std::string_view kName = "MeshRendererPbptMeshExportMapper";

    static bool matches(const core::GameObject& go, const ExportGlobalContext&, pbpt::serde::PbptXmlResult<float>&) {
        const auto* mesh_renderer = go.get_component<component::MeshRenderer>();
        const auto* pbpt_mesh     = go.get_component<component::PbptMesh>();
        if (mesh_renderer == nullptr || pbpt_mesh == nullptr)
            return false;
        if (!mesh_renderer->enabled() || !pbpt_mesh->enabled())
            return false;
        return true;
    }

    static void map(const core::GameObject& go, const ExportGlobalContext& ctx,
                    pbpt::serde::PbptXmlResult<float>& result) {
        const auto* mesh_renderer = go.get_component<component::MeshRenderer>();
        const auto* pbpt_mesh     = go.get_component<component::PbptMesh>();
        const auto* pbpt_light    = go.get_component<component::PbptLight>();

        const auto mesh_handle = pbpt_mesh->mesh_handle();
        if (!mesh_handle.is_valid() || !ctx.resources.alive<rtr::resource::MeshResourceKind>(mesh_handle)) {
            throw std::runtime_error("Pbpt export requires valid and alive mesh handle.");
        }
        const auto& cpu_mesh        = ctx.resources.cpu<rtr::resource::MeshResourceKind>(mesh_handle);
        const auto  object_to_world = compat_export_detail::to_transform(go.node().world_matrix());

        std::string mesh_name = compat_export_detail::make_unique_name(
            "rtr_mesh_" + std::to_string(mesh_handle.value),
            [&](const std::string& name) { return result.scene.resources.mesh_library.name_to_id().contains(name); });
        auto mesh =
            compat_export_detail::to_pbpt_triangle_mesh(cpu_mesh, result.scene.render_transform, object_to_world);
        (void)result.scene.resources.mesh_library.add_item(mesh_name, std::move(mesh));

        const pbpt::math::vec4   base_color = mesh_renderer->base_color();
        const component::PbptRgb reflectance{.r = base_color.x(), .g = base_color.y(), .b = base_color.z()};
        component::validate_pbpt_rgb(reflectance, "MeshRenderer.base_color");

        const std::string material_key = compat_export_detail::reflectance_key(reflectance);
        std::string       material_name;
        if (ctx.material_name_by_reflectance.contains(material_key)) {
            material_name = ctx.material_name_by_reflectance.at(material_key);
        } else {
            material_name = compat_export_detail::make_unique_name(
                "rtr_mat_" + std::to_string(ctx.material_name_by_reflectance.size()), [&](const std::string& name) {
                    return result.scene.resources.any_material_library.name_to_id().contains(name);
                });
            ctx.material_name_by_reflectance[material_key] = material_name;

            auto material =
                pbpt::material::LambertianMaterial<float>(compat_export_detail::rgb_to_piecewise(reflectance));
            (void)result.scene.resources.any_material_library.add_item(material_name, std::move(material));
        }
        result.scene.resources.mesh_material_map[mesh_name] =
            result.scene.resources.any_material_library.name_to_id().at(material_name);

        std::string shape_base = go.name();
        if (shape_base.empty()) {
            shape_base = "go_" + std::to_string(static_cast<std::uint64_t>(go.id()));
        }

        std::unordered_set<std::string> used_shape_ids;
        for (const auto& shape_record : result.scene.resources.shape_instances) {
            used_shape_ids.insert(shape_record.shape_id);
        }
        const std::string shape_id = compat_export_detail::make_unique_shape_id(shape_base, used_shape_ids);

        pbpt::scene::ShapeInstanceRecord<float> shape_record{};
        shape_record.shape_id          = shape_id;
        shape_record.shape_type        = "obj";
        shape_record.mesh_name         = mesh_name;
        shape_record.material_ref_name = material_name;
        shape_record.object_to_world   = object_to_world;

        if (pbpt_light != nullptr && pbpt_light->enabled()) {
            const std::string emission_name =
                compat_export_detail::make_unique_name(shape_id + "_emission", [&](const std::string& name) {
                    return result.scene.resources.reflectance_spectrum_library.name_to_id().contains(name);
                });
            result.scene.resources.reflectance_spectrum_library.add_item(
                emission_name,
                compat_export_detail::to_piecewise_spectrum(pbpt_light->area_emitter().radiance_spectrum));
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
                const std::string light_name = compat_export_detail::make_unique_name(
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
};

}  // namespace rtr::framework::integration
