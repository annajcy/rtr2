#include <pbpt/math/math.h>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

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
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/physics/reset_position.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/system/input/input_types.hpp"

int main() {
    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;
    const pbpt::math::Vec3 kResetPosition{0.0f, 2.0f, 0.0f};
    const pbpt::math::Vec3 kInitialAngularVelocity{0.0f, 4.5f, 0.0f};
    const float            kThresholdY = -1.0f;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth, .window_height = kHeight, .window_title = "RTR Gravity Spin Editor Demo"});

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

        auto& scene = runtime.world().create_scene("gravity_spin_scene");

        auto& camera_go       = scene.create_game_object("main_camera");
        auto& camera          = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 1.6f, 8.0f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 0.8f, 0.0f});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({2.0f, 5.0f, 3.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.95f, 0.88f});
        point_light.set_intensity(60.0f);
        point_light.set_range(30.0f);

        auto&      bunny_go   = scene.create_game_object("bunny_spin");
        const auto bunny_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/stanford_bunny.obj");
        (void)bunny_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), bunny_mesh, pbpt::math::Vec4{0.90f, 0.85f, 0.78f, 1.0f});
        bunny_go.node().set_local_position(kResetPosition);
        bunny_go.node().set_local_scale({10.0f, 10.0f, 10.0f});

        auto& rigid_body = bunny_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        pbpt::math::Mat3 inverse_inertia_tensor_ref = pbpt::math::Mat3::zeros();
        inverse_inertia_tensor_ref[0][0] = 0.6f;
        inverse_inertia_tensor_ref[1][1] = 1.0f;
        inverse_inertia_tensor_ref[2][2] = 0.6f;
        rigid_body.set_inverse_inertia_tensor_ref(inverse_inertia_tensor_ref);

        auto& reset_position = bunny_go.add_component<rtr::framework::component::ResetPosition>();
        reset_position.set_threshold_y(kThresholdY);
        reset_position.set_reset_position(kResetPosition);

        auto& physics_body = runtime.physics_world().get_rigid_body(rigid_body.rigid_body_id());
        physics_body.state().rotation.angular_velocity = kInitialAngularVelocity;

        auto&      ground_go   = scene.create_game_object("ground");
        const auto ground_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");
        (void)ground_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), ground_mesh, pbpt::math::Vec4{0.25f, 0.25f, 0.28f, 1.0f});
        ground_go.node().set_local_position({0.0f, -1.0f, 0.0f});
        ground_go.node().set_local_rotation(
            pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}));
        ground_go.node().set_local_scale({15.0f, 15.0f, 1.0f});

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
