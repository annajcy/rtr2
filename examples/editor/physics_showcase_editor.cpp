#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <stdexcept>

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
    constexpr double   kResetIntervalSeconds = 4.0;

    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR Physics Showcase",
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

        auto& scene = runtime.world().create_scene("physics_showcase_scene");

        auto& camera_go       = scene.create_game_object("main_camera");
        auto& camera          = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 4.2f, 13.5f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

        auto& key_light_go = scene.create_game_object("key_light");
        key_light_go.node().set_local_position({4.0f, 8.0f, 6.0f});
        auto& key_light = key_light_go.add_component<rtr::framework::component::light::PointLight>();
        key_light.set_color({1.0f, 0.95f, 0.90f});
        key_light.set_intensity(90.0f);
        key_light.set_range(60.0f);

        auto& fill_light_go = scene.create_game_object("fill_light");
        fill_light_go.node().set_local_position({-6.0f, 6.0f, 2.0f});
        auto& fill_light = fill_light_go.add_component<rtr::framework::component::light::PointLight>();
        fill_light.set_color({0.70f, 0.80f, 1.0f});
        fill_light.set_intensity(35.0f);
        fill_light.set_range(50.0f);

        const auto quad_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");
        const auto sphere_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "pbpt_scene/cbox/meshes/uv_sphere.obj");

        auto add_panel = [&](const std::string& name,
                             const pbpt::math::Vec3& position,
                             const pbpt::math::Quat& rotation,
                             const pbpt::math::Vec3& scale,
                             const pbpt::math::Vec4& color,
                             const pbpt::math::Vec3& half_extents,
                             rtr::system::physics::RigidBodyType body_type = rtr::system::physics::RigidBodyType::Static,
                             float restitution = 0.0f,
                             float friction = 0.0f) -> rtr::framework::component::RigidBody& {
            auto& go = scene.create_game_object(name);
            go.node().set_local_position(position);
            go.node().set_local_rotation(rotation);
            go.node().set_local_scale(scale);
            (void)go.add_component<rtr::framework::component::MeshRenderer>(runtime.resource_manager(), quad_mesh, color);
            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
            rigid_body.set_type(body_type);
            rigid_body.set_restitution(restitution);
            rigid_body.set_friction(friction);
            (void)go.add_component<rtr::framework::component::BoxCollider>(runtime.physics_world(), half_extents);
            return rigid_body;
        };

        (void)add_panel("ground",
                        pbpt::math::Vec3{0.0f, -1.35f, 0.0f},
                        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
                        pbpt::math::Vec3{16.0f, 9.0f, 1.0f},
                        pbpt::math::Vec4{0.17f, 0.19f, 0.22f, 1.0f},
                        pbpt::math::Vec3{0.5f, 0.5f, 0.06f},
                        rtr::system::physics::RigidBodyType::Static,
                        0.0f,
                        0.55f);

        (void)add_panel("ramp",
                        pbpt::math::Vec3{-4.3f, 0.1f, 0.0f},
                        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}) *
                            pbpt::math::angle_axis(pbpt::math::radians(18.0f), pbpt::math::Vec3{0.0f, 0.0f, 1.0f}),
                        pbpt::math::Vec3{4.5f, 2.4f, 1.0f},
                        pbpt::math::Vec4{0.22f, 0.26f, 0.30f, 1.0f},
                        pbpt::math::Vec3{0.5f, 0.5f, 0.08f},
                        rtr::system::physics::RigidBodyType::Static,
                        0.02f,
                        0.90f);

        (void)add_panel("bounce_pad",
                        pbpt::math::Vec3{-0.4f, -0.4f, 0.0f},
                        pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}),
                        pbpt::math::Vec3{2.8f, 2.0f, 1.0f},
                        pbpt::math::Vec4{0.27f, 0.24f, 0.20f, 1.0f},
                        pbpt::math::Vec3{0.5f, 0.5f, 0.08f},
                        rtr::system::physics::RigidBodyType::Static,
                        0.90f,
                        0.02f);

        (void)add_panel("impact_backdrop",
                        pbpt::math::Vec3{2.7f, 1.0f, -1.3f},
                        pbpt::math::Quat::identity(),
                        pbpt::math::Vec3{3.4f, 3.6f, 1.0f},
                        pbpt::math::Vec4{0.18f, 0.21f, 0.24f, 1.0f},
                        pbpt::math::Vec3{0.5f, 0.5f, 0.08f});

        auto& ramp_ball_go = scene.create_game_object("ramp_ball");
        ramp_ball_go.node().set_local_scale({0.35f, 0.35f, 0.35f});
        (void)ramp_ball_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), sphere_mesh, pbpt::math::Vec4{0.95f, 0.66f, 0.26f, 1.0f});
        auto& ramp_ball = ramp_ball_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        ramp_ball.set_friction(0.85f);
        ramp_ball.set_restitution(0.05f);
        ramp_ball.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(1.0f, 1.0f, 1.0f));
        (void)ramp_ball_go.add_component<rtr::framework::component::SphereCollider>(runtime.physics_world(), 1.0f);

        auto& bounce_ball_go = scene.create_game_object("bounce_ball");
        bounce_ball_go.node().set_local_scale({0.42f, 0.42f, 0.42f});
        (void)bounce_ball_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), sphere_mesh, pbpt::math::Vec4{0.33f, 0.80f, 0.93f, 1.0f});
        auto& bounce_ball = bounce_ball_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        bounce_ball.set_restitution(0.88f);
        bounce_ball.set_friction(0.02f);
        bounce_ball.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(1.0f, 1.0f, 1.0f));
        (void)bounce_ball_go.add_component<rtr::framework::component::SphereCollider>(runtime.physics_world(), 1.0f);

        auto& target_ball_go = scene.create_game_object("impact_target_ball");
        target_ball_go.node().set_local_scale({0.44f, 0.44f, 0.44f});
        (void)target_ball_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), sphere_mesh, pbpt::math::Vec4{0.43f, 0.92f, 0.57f, 1.0f});
        auto& target_ball = target_ball_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        target_ball.set_use_gravity(false);
        target_ball.set_restitution(0.35f);
        target_ball.set_friction(0.08f);
        target_ball.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(1.0f, 1.0f, 1.0f));
        (void)target_ball_go.add_component<rtr::framework::component::SphereCollider>(runtime.physics_world(), 1.0f);

        auto& striker_go = scene.create_game_object("impact_striker");
        striker_go.node().set_local_scale({0.5f, 1.4f, 1.0f});
        (void)striker_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), quad_mesh, pbpt::math::Vec4{0.93f, 0.37f, 0.48f, 1.0f});
        auto& striker = striker_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        striker.set_use_gravity(false);
        striker.set_restitution(0.70f);
        striker.set_friction(0.10f);
        striker.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(0.0f, 0.0f, 1.8f));
        (void)striker_go.add_component<rtr::framework::component::BoxCollider>(
            runtime.physics_world(), pbpt::math::Vec3{0.5f, 0.5f, 0.10f});

        auto& spinner_go = scene.create_game_object("torque_spinner");
        spinner_go.node().set_local_scale({0.55f, 2.2f, 1.0f});
        (void)spinner_go.add_component<rtr::framework::component::MeshRenderer>(
            runtime.resource_manager(), quad_mesh, pbpt::math::Vec4{0.76f, 0.56f, 0.95f, 1.0f});
        auto& spinner = spinner_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_world());
        spinner.set_use_gravity(false);
        spinner.set_friction(0.0f);
        spinner.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(0.2f, 1.5f, 0.2f));
        (void)spinner_go.add_component<rtr::framework::component::BoxCollider>(
            runtime.physics_world(), pbpt::math::Vec3{0.5f, 0.5f, 0.08f});

        auto reset_showcase = [&]() {
            reset_dynamic_body(ramp_ball_go, ramp_ball, runtime.physics_world(), pbpt::math::Vec3{-5.2f, 1.7f, 0.0f},
                               pbpt::math::Quat::identity(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

            reset_dynamic_body(bounce_ball_go, bounce_ball, runtime.physics_world(), pbpt::math::Vec3{-0.4f, 2.3f, 0.0f},
                               pbpt::math::Quat::identity(), pbpt::math::Vec3{0.25f, 0.0f, 0.0f});

            reset_dynamic_body(target_ball_go, target_ball, runtime.physics_world(), pbpt::math::Vec3{2.4f, 0.9f, 0.0f},
                               pbpt::math::Quat::identity(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f});

            reset_dynamic_body(striker_go, striker, runtime.physics_world(), pbpt::math::Vec3{0.5f, 1.45f, 0.0f},
                               pbpt::math::Quat::identity(), pbpt::math::Vec3{4.8f, -0.15f, 0.0f});

            reset_dynamic_body(spinner_go, spinner, runtime.physics_world(), pbpt::math::Vec3{5.0f, 1.2f, 0.0f},
                               pbpt::math::Quat::identity(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f},
                               pbpt::math::Vec3{0.0f, 0.6f, 0.0f});
        };

        double reset_timer = 0.0;
        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_startup =
                [&](rtr::app::RuntimeContext&) {
                    reset_showcase();
                    std::cout << "Physics showcase loaded.\n"
                              << "Left: gravity + slope friction rolling.\n"
                              << "Center-left: high restitution bounce.\n"
                              << "Center-right: off-center impact causing rotation.\n"
                              << "Right: continuous torque-driven spin.\n";
                },
            .on_pre_update =
                [&](rtr::app::RuntimeContext& ctx) {
                    reset_timer += ctx.delta_seconds;
                    if (reset_timer >= kResetIntervalSeconds) {
                        reset_timer = 0.0;
                        reset_showcase();
                    }

                    spinner.add_torque(pbpt::math::Vec3{0.0f, 7.0f, 0.0f});
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
