#pragma once

#include <stdexcept>

#include <glm/gtc/matrix_inverse.hpp>

#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/render/forward_scene_view.hpp"

namespace rtr::system::render {

inline ForwardSceneView build_forward_scene_view(
    const framework::core::Scene& scene,
    resource::ResourceManager& resources
) {
    const auto* active_camera = scene.active_camera();
    if (active_camera == nullptr) {
        throw std::runtime_error("Active scene does not have an active camera.");
    }

    ForwardSceneView view{};
    view.camera.view = active_camera->view_matrix();
    view.camera.proj = active_camera->projection_matrix();

    const auto active_nodes = scene.scene_graph().active_nodes();
    view.renderables.reserve(active_nodes.size());

    for (const auto id : active_nodes) {
        const auto* go = scene.find_game_object(id);
        if (go == nullptr) {
            continue;
        }
        const auto* mesh_renderer = go->get_component<framework::component::MeshRenderer>();
        if (mesh_renderer == nullptr) {
            continue;
        }

        const auto node = scene.scene_graph().node(id);
        const glm::mat4 model = node.world_matrix();
        const glm::mat4 normal = glm::transpose(glm::inverse(model));

        view.renderables.emplace_back(ForwardSceneRenderable{
            .instance_id = static_cast<std::uint64_t>(id),
            .mesh = resources.load_mesh(mesh_renderer->mesh_path()),
            .albedo_texture = resources.load_texture(mesh_renderer->albedo_texture_path()),
            .model = model,
            .normal = normal
        });
    }

    return view;
}

} // namespace rtr::system::render
