#include <pbpt/math/math.h>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "angry_bunny_controller.hpp"
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
#include "rtr/framework/component/physics/rigid_body/mesh_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/system/input/input_types.hpp"

int main() {
    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;
    const pbpt::math::Vec3 kResetPosition{0.0f, 0.6f, 0.0f};
    const pbpt::math::Vec3 kLaunchVelocity{3.5f, 5.0f, 0.0f};
    const pbpt::math::Vec3 kInitialAngularVelocity{0.0f, 4.5f, 0.0f};

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "GAMES103 Lab1 Angry Bunny",
        });

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

        auto& scene = runtime.world().create_scene("games103_lab1_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 1.8f, 8.5f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({2.0f, 5.5f, 4.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.96f, 0.90f});
        point_light.set_intensity(60.0f);
        point_light.set_range(35.0f);

        const auto bunny_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/stanford_bunny.obj");
        const auto quad_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");

        auto& bunny_go = scene.create_game_object("angry_bunny");
        bunny_go.node().set_local_position(kResetPosition);
        bunny_go.node().set_local_scale({10.0f, 10.0f, 10.0f});
        (void)bunny_go.add_component<rtr::framework::component::StaticMeshComponent>(
            runtime.resource_manager(), bunny_mesh, pbpt::math::Vec4{0.90f, 0.85f, 0.78f, 1.0f});
        auto& bunny_body = bunny_go.add_component<rtr::framework::component::RigidBody>(
            runtime.physics_system().rigid_body_world(), 1.0f, rtr::system::physics::RigidBodyType::Dynamic, false,
            pbpt::math::Mat3::zeros(), 0.2f, 1.2f, 0.99f, 0.985f);
        pbpt::math::Mat3 inverse_inertia_tensor = pbpt::math::Mat3::zeros();
        inverse_inertia_tensor[0][0] = 0.6f;
        inverse_inertia_tensor[1][1] = 1.0f;
        inverse_inertia_tensor[2][2] = 0.6f;
        bunny_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor);
        (void)bunny_go.add_component<rtr::framework::component::MeshCollider>(runtime.physics_system().rigid_body_world());
        (void)bunny_go.add_component<rtr::examples::games103_lab::lab1_angry_bunny::AngryBunnyController>(
            runtime.input_system().state(), kLaunchVelocity, kInitialAngularVelocity, kResetPosition);

        auto add_plane = [&](const char* name,
                             const pbpt::math::Vec3& position,
                             const pbpt::math::Quat& rotation,
                             const pbpt::math::Vec3& scale,
                             const pbpt::math::Vec3& normal_local,
                             const pbpt::math::Vec4& color) {
            auto& go = scene.create_game_object(name);
            go.node().set_local_position(position);
            go.node().set_local_rotation(rotation);
            go.node().set_local_scale(scale);
            (void)go.add_component<rtr::framework::component::StaticMeshComponent>(runtime.resource_manager(), quad_mesh, color);
            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>(runtime.physics_system().rigid_body_world());
            rigid_body.set_type(rtr::system::physics::RigidBodyType::Static);
            (void)go.add_component<rtr::framework::component::PlaneCollider>(runtime.physics_system().rigid_body_world(), normal_local);
        };

        add_plane("ground",
                  pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
                  pbpt::math::Vec3{15.0f, 15.0f, 1.0f},
                  pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
                  pbpt::math::Vec4{0.24f, 0.28f, 0.31f, 1.0f});
        add_plane("left_wall",
                  pbpt::math::Vec3{-2.8f, 0.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
                  pbpt::math::Vec3{10.0f, 3.0f, 1.0f},
                  pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
                  pbpt::math::Vec4{0.20f, 0.24f, 0.27f, 1.0f});
        add_plane("right_wall",
                  pbpt::math::Vec3{2.8f, 0.0f, 0.0f},
                  pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
                  pbpt::math::Vec3{10.0f, 3.0f, 1.0f},
                  pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
                  pbpt::math::Vec4{0.20f, 0.24f, 0.27f, 1.0f});

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
