#include <cstdlib>
#include <array>
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
#include "rtr/framework/component/physics/rigid_body/plane_collider.hpp"
#include "rtr/framework/component/physics/rigid_body/rigid_body.hpp"
#include "rtr/framework/component/physics/rigid_body/sphere_collider.hpp"
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
                        rtr::system::physics::rb::RigidBodySystem& physics_world,
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
    constexpr double   kResetIntervalSeconds = 6.0;
    constexpr std::size_t kStackBoxCount     = 5;
    constexpr float kCubeInverseInertia      = 7.4f;

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
        camera_go.node().set_local_position({0.8f, 5.2f, 13.5f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 2.8f, 0.0f});

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
        const auto cube_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/cube.obj");
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
            (void)go.add_component<rtr::framework::component::StaticMeshComponent>(runtime.resource_manager(), quad_mesh, color);
            auto& rigid_body = go.add_component<rtr::framework::component::RigidBody>(runtime.physics_system().rigid_body_system());
            rigid_body.set_type(rtr::system::physics::rb::RigidBodyType::Static);
            rigid_body.set_restitution(restitution);
            rigid_body.set_friction(friction);
            (void)go.add_component<rtr::framework::component::PlaneCollider>(
                runtime.physics_system().rigid_body_system(), pbpt::math::Vec3{0.0f, 0.0f, 1.0f});
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
        (void)sphere_go.add_component<rtr::framework::component::StaticMeshComponent>(
            runtime.resource_manager(), sphere_mesh, pbpt::math::Vec4{0.95f, 0.66f, 0.26f, 1.0f});
        auto& sphere_body = sphere_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_system().rigid_body_system());
        sphere_body.set_restitution(0.8f);
        sphere_body.set_friction(0.86f);
        sphere_body.set_inverse_inertia_tensor_ref(diagonal_inverse_inertia(1.0f, 1.0f, 1.0f));
        (void)sphere_go.add_component<rtr::framework::component::SphereCollider>(runtime.physics_system().rigid_body_system(), 1.0f);

        struct StackBoxEntry {
            rtr::framework::core::GameObject*            game_object{nullptr};
            rtr::framework::component::RigidBody*        rigid_body{nullptr};
        };

        const std::array<pbpt::math::Vec4, kStackBoxCount> stack_colors{{
            pbpt::math::Vec4{0.34f, 0.77f, 0.93f, 1.0f},
            pbpt::math::Vec4{0.48f, 0.84f, 0.61f, 1.0f},
            pbpt::math::Vec4{0.95f, 0.72f, 0.30f, 1.0f},
            pbpt::math::Vec4{0.92f, 0.50f, 0.46f, 1.0f},
            pbpt::math::Vec4{0.73f, 0.61f, 0.93f, 1.0f},
        }};

        std::vector<StackBoxEntry> stack_boxes;
        stack_boxes.reserve(kStackBoxCount);
        for (std::size_t index = 0; index < kStackBoxCount; ++index) {
            auto& box_go = scene.create_game_object("stack_box_" + std::to_string(index));
            box_go.node().set_local_scale({0.9f, 0.9f, 0.9f});
            (void)box_go.add_component<rtr::framework::component::StaticMeshComponent>(
                runtime.resource_manager(), cube_mesh, stack_colors[index]);

            auto& box_body =
                box_go.add_component<rtr::framework::component::RigidBody>(runtime.physics_system().rigid_body_system(), 1.0f);
            box_body.set_mass(1.0f);
            box_body.set_restitution(0.02f);
            box_body.set_friction(0.18f);
            box_body.set_linear_decay(0.985f);
            box_body.set_angular_decay(0.90f);
            box_body.set_inverse_inertia_tensor_ref(
                diagonal_inverse_inertia(kCubeInverseInertia, kCubeInverseInertia, kCubeInverseInertia));
            (void)box_go.add_component<rtr::framework::component::BoxCollider>(
                runtime.physics_system().rigid_body_system(), pbpt::math::Vec3{0.5f, 0.5f, 0.5f});

            stack_boxes.push_back(StackBoxEntry{.game_object = &box_go, .rigid_body = &box_body});
        }

        auto reset_demo = [&]() {
            reset_dynamic_body(sphere_go,
                               sphere_body,
                               runtime.physics_system().rigid_body_system(),
                               pbpt::math::Vec3{-4.0f, 1.9f, 0.75f},
                               pbpt::math::Quat::identity(),
                               pbpt::math::Vec3{2.9f, -0.2f, -0.1f});

            const pbpt::math::Vec3 stack_base_position{-1.9f, 4.4f, -0.2f};
            constexpr float kVerticalSpacing = 0.98f;

            for (std::size_t index = 0; index < stack_boxes.size(); ++index) {
                const float index_f = static_cast<float>(index);
                const float x_offset = (index % 2 == 0) ? 0.08f : -0.08f;
                const float z_offset = (index % 2 == 0) ? 0.12f : -0.12f;
                const float x_rotation_deg = (index % 2 == 0) ? 11.0f : -11.0f;
                const float z_rotation_deg = (index % 2 == 0) ? 8.0f : -8.0f;
                const float y_rotation_deg = -4.0f + 2.0f * index_f;
                const float angular_speed_x = (index % 2 == 0) ? 0.55f : -0.55f;
                const float angular_speed_z = (index % 2 == 0) ? 0.22f : -0.22f;
                const float angular_speed_y = -0.08f + 0.04f * index_f;
                const auto position = pbpt::math::Vec3{
                    stack_base_position.x() + x_offset,
                    stack_base_position.y() + index_f * kVerticalSpacing,
                    stack_base_position.z() + z_offset,
                };
                const auto orientation =
                    pbpt::math::angle_axis(pbpt::math::radians(y_rotation_deg), pbpt::math::Vec3{0.0f, 1.0f, 0.0f}) *
                    pbpt::math::angle_axis(pbpt::math::radians(x_rotation_deg), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}) *
                    pbpt::math::angle_axis(pbpt::math::radians(z_rotation_deg), pbpt::math::Vec3{0.0f, 0.0f, 1.0f});

                reset_dynamic_body(*stack_boxes[index].game_object,
                                   *stack_boxes[index].rigid_body,
                                   runtime.physics_system().rigid_body_system(),
                                   position,
                                   orientation,
                                   pbpt::math::Vec3{0.0f},
                                   pbpt::math::Vec3{angular_speed_x, angular_speed_y, angular_speed_z});
            }
        };

        double reset_timer = 0.0;
        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_startup =
                [&](rtr::app::RuntimeContext&) {
                    reset_demo();
                    std::cout << "Plane collision demo loaded.\n"
                              << "Sphere: sphere-plane contact on the ramp and wall.\n"
                              << "Boxes: five stacked cubes fall from height and collide with each other and the planes.\n";
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
