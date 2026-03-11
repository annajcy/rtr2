#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

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
#include "rtr/framework/component/material/mesh_renderer.hpp"
#include "rtr/framework/component/physics/box_collider.hpp"
#include "rtr/framework/component/physics/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body.hpp"
#include "rtr/framework/component/physics/sphere_collider.hpp"
#include "rtr/system/input/input_types.hpp"

namespace {

pbpt::math::Mat3 diagonal_inverse_inertia(float x, float y, float z) {
    pbpt::math::Mat3 result = pbpt::math::Mat3::zeros();
    result[0][0]            = x;
    result[1][1]            = y;
    result[2][2]            = z;
    return result;
}

void sync_camera_aspect(rtr::app::RuntimeContext& ctx) {
    auto* active_scene = ctx.world.active_scene();
    if (active_scene == nullptr) {
        throw std::runtime_error("No active scene.");
    }

    auto* active_camera =
        active_scene->find_game_object("main_camera")->get_component<rtr::framework::component::Camera>();
    if (active_camera == nullptr) {
        return;
    }

    const auto [fb_w, fb_h] = ctx.renderer.window().framebuffer_size();
    if (fb_w <= 0 || fb_h <= 0) {
        return;
    }

    if (auto* perspective = dynamic_cast<rtr::framework::component::PerspectiveCamera*>(active_camera);
        perspective != nullptr) {
        perspective->aspect_ratio() = static_cast<float>(fb_w) / static_cast<float>(fb_h);
    }
}

void reset_dynamic_body(rtr::framework::core::GameObject& game_object,
                        rtr::framework::component::RigidBody& rigid_body,
                        rtr::system::physics::PhysicsWorld& physics_world,
                        const pbpt::math::Vec3& position,
                        const pbpt::math::Quat& orientation,
                        const pbpt::math::Vec3& linear_velocity,
                        const pbpt::math::Vec3& angular_velocity = pbpt::math::Vec3{0.0f}) {
    game_object.node().set_local_position(position);
    game_object.node().set_local_rotation(orientation);

    rigid_body.set_position(position);
    rigid_body.set_orientation(orientation);
    rigid_body.reset_dynamics();

    auto& body = physics_world.get_rigid_body(rigid_body.rigid_body_id());
    body.state().translation.linear_velocity = linear_velocity;
    body.state().rotation.angular_velocity   = angular_velocity;
}

}  // namespace

int main() {
    constexpr uint32_t kWidth  = 1440;
    constexpr uint32_t kHeight = 900;
    constexpr double   kResetIntervalSeconds = 4.5;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR Plane Collision Demo",
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

        auto& scene = runtime.world().create_scene("plane_collision_scene");

        auto& camera_go       = scene.create_game_object("main_camera");
        auto& camera          = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.6f, 3.8f, 11.5f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.8f, 0.8f, 0.0f});

        auto& key_light_go = scene.create_game_object("key_light");
        key_light_go.node().set_local_position({4.0f, 8.0f, 5.0f});
        auto& key_light = key_light_go.add_component<rtr::framework::component::light::PointLight>();
        key_light.set_color({1.0f, 0.96f, 0.90f});
        key_light.set_intensity(90.0f);
        key_light.set_range(60.0f);

        auto& fill_light_go = scene.create_game_object("fill_light");
        fill_light_go.node().set_local_position({-6.0f, 5.0f, 3.0f});
        auto& fill_light = fill_light_go.add_component<rtr::framework::component::light::PointLight>();
        fill_light.set_color({0.72f, 0.84f, 1.0f});
        fill_light.set_intensity(30.0f);
        fill_light.set_range(40.0f);

        const auto quad_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");
        const auto sphere_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "pbpt_scene/cbox/meshes/uv_sphere.obj");

        auto add_plane = [&](const std::string& name,
                             const pbpt::math::Vec3& position,
                             const pbpt::math::Quat& rotation,
                             const pbpt::math::Vec3& scale,
                             const pbpt::math::Vec4& color,
                             float restitution,
                             float friction) {
            auto& go = scene.create_game_object(name);
            go.node().set_local_position(position);
            go.node().set_local_rotation(rotation);
            go.node().set_local_scale(scale);
            (void)go.add_component<rtr::framework::component::MeshRenderer>(runtime.resource_manager(), quad_mesh, color);
            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
            rigid_body.set_type(rtr::system::physics::RigidBodyType::Static);
            rigid_body.set_restitution(restitution);
            rigid_body.set_friction(friction);
            (void)go.add_component<rtr::framework::component::PlaneCollider>(
                runtime.physics_world(), pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
        };

        (void)add_plane("ramp_plane",
                        pbpt::math::Vec3{-0.9f, -1.0f, 0.0f},
                        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}) *
                            pbpt::math::angle_axis(pbpt::math::radians(14.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
                        pbpt::math::Vec3{14.0f, 8.0f, 1.0f},
                        pbpt::math::Vec4{0.18f, 0.21f, 0.24f, 1.0f},
                        0.02f,
                        0.72f);
        (void)add_plane("backstop_plane",
                        pbpt::math::Vec3{4.5f, 0.7f, 0.0f},
                        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}),
                        pbpt::math::Vec3{6.0f, 4.0f, 1.0f},
                        pbpt::math::Vec4{0.24f, 0.28f, 0.32f, 1.0f},
                        0.10f,
                        0.25f);

        auto& sphere_go = scene.create_game_object("rolling_sphere");
        sphere_go.node().set_local_scale({0.48f, 0.48f, 0.48f});
        (void)sphere_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), sphere_mesh, pbpt::math::Vec4{0.95f, 0.66f, 0.26f, 1.0f});
        auto& sphere_body = sphere_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        sphere_body.set_restitution(0.08f);
        sphere_body.set_friction(0.86f);
        sphere_body.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(1.0f, 1.0f, 1.0f));
        (void)sphere_go.add_component<rtr::framework::component::SphereCollider>(runtime.physics_world(), 1.0f);

        auto& box_go = scene.create_game_object("sliding_box");
        box_go.node().set_local_scale({1.45f, 0.95f, 1.0f});
        (void)box_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), quad_mesh, pbpt::math::Vec4{0.38f, 0.78f, 0.94f, 1.0f});
        auto& box_body = box_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        box_body.set_restitution(0.03f);
        box_body.set_friction(0.50f);
        box_body.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(0.8f, 1.2f, 1.8f));
        (void)box_go.add_component<rtr::framework::component::BoxCollider>(
            runtime.physics_world(), pbpt::math::Vec3{0.72f, 0.48f, 0.12f});

        auto reset_demo = [&]() {
            reset_dynamic_body(sphere_go,
                               sphere_body,
                               runtime.physics_world(),
                               pbpt::math::Vec3{-4.0f, 1.9f, 0.75f},
                               pbpt::math::Quat::identity(),
                               pbpt::math::Vec3{2.9f, -0.2f, -0.1f});

            reset_dynamic_body(box_go,
                               box_body,
                               runtime.physics_world(),
                               pbpt::math::Vec3{-2.3f, 2.8f, -0.7f},
                               pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}) *
                                   pbpt::math::angle_axis(pbpt::math::radians(22.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
                               pbpt::math::Vec3{2.2f, -0.1f, 0.0f},
                               pbpt::math::Vec3{0.0f, 0.0f, 0.6f});
        };

        double reset_timer = 0.0;
        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_startup =
                [&](rtr::app::RuntimeContext&) {
                    reset_demo();
                    std::cout << "Plane collision demo loaded.\n"
                              << "Sphere: sphere-plane contact on the ramp and wall.\n"
                              << "Box: box-plane contact with angular response on the same planes.\n";
                },
            .on_pre_update =
                [&](rtr::app::RuntimeContext& ctx) {
                    reset_timer += ctx.delta_seconds;
                    if (reset_timer >= kResetIntervalSeconds) {
                        reset_timer = 0.0;
                        reset_demo();
                    }
                },
            .on_post_update =
                [editor_host](rtr::app::RuntimeContext& ctx) {
                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial  = ctx.frame_serial,
                        .delta_seconds = ctx.delta_seconds,
                        .paused        = ctx.paused,
                    });
                    sync_camera_aspect(ctx);
                },
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input_system.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        ctx.renderer.window().close();
                    }
                },
        });

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
