#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <pbpt/math/math.h>

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/render/forward_editor_pipeline.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/component/physics/rigid_body/box_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
#include "rtr/system/input/input_types.hpp"

namespace {

struct SphereSpawn {
    pbpt::math::Vec3 position;
    pbpt::math::Vec3 velocity;
    pbpt::math::Vec4 color;
};

}  // namespace

int main() {
    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth, .window_height = kHeight, .window_title = "RTR Sphere Pen Collision Demo"});

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

        auto& scene = runtime.world().create_scene("sphere_pen_collision_scene");

        auto& camera_go       = scene.create_game_object("main_camera");
        auto& camera          = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 3.0f, 10.0f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({3.0f, 8.0f, 4.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.96f, 0.92f});
        point_light.set_intensity(70.0f);
        point_light.set_range(50.0f);

        const auto quad_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");
        const auto sphere_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "pbpt_scene/cbox/meshes/uv_sphere.obj");

        auto add_panel = [&](const std::string& name, const pbpt::math::Vec3& position, const pbpt::math::Quat& rotation,
                             const pbpt::math::Vec3& scale, const pbpt::math::Vec4& color) {
            auto& go = scene.create_game_object(name);
            go.node().set_local_position(position);
            go.node().set_local_rotation(rotation);
            go.node().set_local_scale(scale);
            (void)go.add_component<rtr::framework::component::StaticMeshComponent>(runtime.resource_manager(), quad_mesh, color);
            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>();
            rigid_body.set_type(rtr::system::physics::rb::RigidBodyType::Static);
            (void)go.add_component<rtr::framework::component::BoxCollider>(pbpt::math::Vec3{0.5f, 0.5f, 0.05f});
        };

        add_panel("floor", pbpt::math::Vec3{0.0f, -1.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
                  pbpt::math::Vec3{8.0f, 8.0f, 1.0f}, pbpt::math::Vec4{0.22f, 0.25f, 0.28f, 1.0f});
        add_panel("back_wall", pbpt::math::Vec3{0.0f, 0.0f, -4.0f}, pbpt::math::Quat::identity(),
                  pbpt::math::Vec3{8.0f, 2.0f, 1.0f}, pbpt::math::Vec4{0.25f, 0.29f, 0.33f, 1.0f});
        // add_panel("front_wall", pbpt::math::Vec3{0.0f, 0.0f, 4.0f},
        //           pbpt::math::angle_axis(pbpt::math::radians(180.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
        //           pbpt::math::Vec3{8.0f, 2.0f, 1.0f}, pbpt::math::Vec4{0.25f, 0.29f, 0.33f, 1.0f});
        add_panel("left_wall", pbpt::math::Vec3{-4.0f, 0.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
                  pbpt::math::Vec3{8.0f, 2.0f, 1.0f}, pbpt::math::Vec4{0.20f, 0.24f, 0.27f, 1.0f});
        add_panel("right_wall", pbpt::math::Vec3{4.0f, 0.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
                  pbpt::math::Vec3{8.0f, 2.0f, 1.0f}, pbpt::math::Vec4{0.20f, 0.24f, 0.27f, 1.0f});

        const std::vector<SphereSpawn> spheres = {
            {{-1.2f, 0.5f, -1.0f}, { 1.0f, 0.0f,  0.7f}, {0.90f, 0.35f, 0.30f, 1.0f}},
            {{ 1.1f, 0.8f, -0.3f}, {-0.8f, 0.0f,  0.9f}, {0.95f, 0.72f, 0.28f, 1.0f}},
            {{-0.8f, 1.1f,  1.2f}, { 0.7f, 0.0f, -1.0f}, {0.35f, 0.78f, 0.92f, 1.0f}},
            {{ 0.9f, 1.4f,  0.8f}, {-1.1f, 0.0f, -0.4f}, {0.48f, 0.92f, 0.55f, 1.0f}},
            {{ 0.0f, 1.7f,  0.0f}, { 0.6f, 0.0f,  0.2f}, {0.82f, 0.48f, 0.92f, 1.0f}},
        };

        for (std::size_t i = 0; i < spheres.size(); ++i) {
            const auto& spawn = spheres[i];
            auto&       go    = scene.create_game_object("sphere_" + std::to_string(i));
            go.node().set_local_position(spawn.position);
            go.node().set_local_scale({0.2f, 0.2f, 0.2f});
            (void)go.add_component<rtr::framework::component::StaticMeshComponent>(runtime.resource_manager(), sphere_mesh, spawn.color);

            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>();
            (void)go.add_component<rtr::framework::component::SphereCollider>(1.0f);
            rigid_body.set_linear_velocity(spawn.velocity);
        }

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

                    auto* active_camera =
                        active_scene->find_game_object("main_camera")->get_component<rtr::framework::component::Camera>();
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
                    if (ctx.input_system.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
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
