#include <pbpt/math/math.h>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "pbpt/math/vector.hpp"
#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/editor/render/forward_editor_pipeline.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"

namespace {

rtr::framework::component::Camera* find_unique_active_camera(rtr::framework::core::Scene& scene) {
    rtr::framework::component::Camera* active_camera = nullptr;
    for (const auto node_id : scene.scene_graph().active_nodes()) {
        auto* go = scene.find_game_object(node_id);
        if (go == nullptr || !go->enabled()) {
            continue;
        }
        auto* camera = go->get_component<rtr::framework::component::Camera>();
        if (camera == nullptr || !camera->enabled() || !camera->active()) {
            continue;
        }
        if (active_camera != nullptr) {
            return nullptr;
        }
        active_camera = camera;
    }
    return active_camera;
}

}  // namespace

int main() {
    constexpr uint32_t kWidth             = 1280;
    constexpr uint32_t kHeight            = 720;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{.window_width         = kWidth,
                                                                .window_height        = kHeight,
                                                                .window_title         = "RTR Framework Quickstart"});

        auto editor_host = std::make_shared<rtr::editor::EditorHost>(runtime);
        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());

        auto editor_pipeline = std::make_unique<rtr::editor::render::ForwardEditorPipeline>(
            runtime.renderer().build_pipeline_runtime(), editor_host);
        rtr::editor::bind_input_capture_to_editor(runtime.input_system(), *editor_pipeline);
        runtime.set_pipeline(std::move(editor_pipeline));

        auto& scene = runtime.world().create_scene("main_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera    = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 1.0f, 6.0f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());

        camera.camera_look_at_point_world(pbpt::math::vec3{0.0, 0.0, 0.0});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({2.0f, 4.0f, 4.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.9f, 0.8f});
        point_light.set_intensity(50.0f);
        point_light.set_range(20.0f);

        auto add_mesh_renderer = [&](rtr::framework::core::GameObject& go, const std::string& mesh_path,
                                     const pbpt::math::vec4& base_color) {
            const auto mesh_handle =
                runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(mesh_path);
            (void)go.add_component<rtr::framework::component::MeshRenderer>(mesh_handle, base_color);
        };

        auto& go_a = scene.create_game_object("mesh_a");
        add_mesh_renderer(go_a, "models/spot.obj", pbpt::math::vec4{0.2f, 0.7f, 0.9f, 1.0f});
        go_a.node().set_local_position({-2.5f, 0.0f, 0.0f});

        auto& go_b = scene.create_game_object("mesh_b");
        add_mesh_renderer(go_b, "models/stanford_bunny.obj", pbpt::math::vec4{0.9f, 0.85f, 0.75f, 1.0f});
        go_b.node().set_local_position({0.0f, 0.0f, 0.0f});

        auto& go_c = scene.create_game_object("mesh_c");
        add_mesh_renderer(go_c, "models/colored_quad.obj", pbpt::math::vec4{0.9f, 0.25f, 0.25f, 1.0f});
        go_c.node().set_local_position({2.5f, 0.0f, 0.0f});

        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_post_update =
                [editor_host](rtr::app::RuntimeContext& ctx) {
                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial  = ctx.frame_serial,
                        .delta_seconds = ctx.delta_seconds,
                        .paused        = ctx.paused,
                    });

                    auto* active_scene = ctx.world.active_scene();
                    if (active_scene == nullptr) {
                        throw std::runtime_error("No active scene.");
                    }

                    auto* active_camera = find_unique_active_camera(*active_scene);
                    if (active_camera != nullptr) {
                        const auto [fb_w, fb_h] = ctx.renderer.window().framebuffer_size();
                        if (fb_w > 0 && fb_h > 0) {
                            if (auto* perspective =
                                    dynamic_cast<rtr::framework::component::PerspectiveCamera*>(active_camera);
                                perspective != nullptr) {
                                perspective->aspect_ratio() = static_cast<float>(fb_w) / static_cast<float>(fb_h);
                            }
                        }
                    }
                },
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        ctx.renderer.window().close();
                    }
                }});

        const auto result = runtime.run();
        if (!result.ok) {
            throw std::runtime_error(result.error_message);
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
