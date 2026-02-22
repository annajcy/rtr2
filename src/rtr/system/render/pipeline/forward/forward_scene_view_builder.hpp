#pragma once

#include <pbpt/math/math.h>
#include <stdexcept>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/pipeline/forward/forward_scene_view.hpp"

namespace rtr::system::render {

inline ForwardSceneView build_forward_scene_view(const framework::core::Scene& scene,
                                                 resource::ResourceManager&    resources) {
    const auto* active_camera = scene.active_camera();
    if (active_camera == nullptr) {
        throw std::runtime_error("Active scene does not have an active camera.");
    }

    ForwardSceneView view{};
    view.camera.view      = active_camera->view_matrix();
    view.camera.proj      = active_camera->projection_matrix();
    view.camera.world_pos = active_camera->node().world_position();

    const auto active_nodes = scene.scene_graph().active_nodes();
    view.renderables.reserve(active_nodes.size());

    for (const auto id : active_nodes) {
        const auto* go = scene.find_game_object(id);
        if (go == nullptr) {
            continue;
        }
        const auto* mesh_renderer = go->get_component<framework::component::MeshRenderer>();
        const auto* point_light   = go->get_component<framework::component::light::PointLight>();

        const auto             node  = scene.scene_graph().node(id);
        const pbpt::math::mat4 model = node.world_matrix();

        if (point_light != nullptr && point_light->enabled() && view.point_lights.size() < kMaxPointLights) {
            ForwardScenePointLight pl{};
            pl.position          = node.world_position();
            pl.intensity         = point_light->intensity;
            pl.color             = point_light->color;
            pl.range             = point_light->range;
            pl.specular_strength = point_light->specular_strength;
            pl.shininess         = point_light->shininess;
            view.point_lights.emplace_back(pl);
        }

        if (mesh_renderer == nullptr || !mesh_renderer->enabled()) {
            continue;
        }

        const pbpt::math::mat4     normal      = pbpt::math::transpose(pbpt::math::inverse(model));
        const resource::MeshHandle mesh_handle = mesh_renderer->mesh_handle();
        if (!mesh_handle.is_valid() || !resources.alive<rtr::resource::MeshResourceKind>(mesh_handle)) {
            throw std::runtime_error("MeshRenderer mesh handle is invalid or unloaded.");
        }

        view.renderables.emplace_back(ForwardSceneRenderable{.instance_id = static_cast<std::uint64_t>(id),
                                                             .mesh        = mesh_handle,
                                                             .base_color  = mesh_renderer->base_color(),
                                                             .model       = model,
                                                             .normal      = normal});
    }

    return view;
}

}  // namespace rtr::system::render
