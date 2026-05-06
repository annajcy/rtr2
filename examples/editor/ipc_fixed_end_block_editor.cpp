#include <pbpt/math/math.h>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include "rtr/editor/core/editor_app_runtime.hpp"
#include "rtr/editor/core/editor_capture.hpp"
#include "rtr/editor/core/editor_host.hpp"
#include "rtr/editor/panel/hierarchy_panel.hpp"
#include "rtr/editor/panel/inspector_panel.hpp"
#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/editor/panel/scene_view_panel.hpp"
#include "rtr/editor/panel/stats_panel.hpp"
#include "rtr/editor/render/editor_output_backend.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/camera_control/free_look_camera_controller.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/mesh_tet_converter/tet_to_mesh.hpp"

namespace {

rtr::system::physics::ipc::TetBody make_fixed_end_block() {
    namespace ipc = rtr::system::physics::ipc;

    auto body = ipc::generate_tet_block(2, 6, 6, 0.2, Eigen::Vector3d(-0.6, 1.4, -0.2));

    double min_x = std::numeric_limits<double>::infinity();
    for (const auto& X : body.geometry.rest_positions) {
        min_x = std::min(min_x, X.x());
    }

    body.fixed_vertices.assign(body.geometry.vertex_count(), false);
    for (std::size_t i = 0; i < body.geometry.vertex_count(); ++i) {
        if (std::abs(body.geometry.rest_positions[i].x() - min_x) < 1e-9) {
            body.fixed_vertices[i] = true;
        }
    }

    return body;
}

void register_ipc_tet_object(rtr::framework::core::GameObject& game_object,
                             rtr::editor::EditorAppRuntime& runtime,
                             rtr::system::physics::ipc::TetBody body,
                             const pbpt::math::Vec4& color) {
    auto& ipc_component = game_object.add_component<rtr::framework::component::IPCTetComponent>(std::move(body));
    const rtr::utils::ObjMeshData initial_mesh = ipc_component.mesh_cache();
    const auto mesh_handle = runtime.resource_manager()
        .create<rtr::resource::DeformableMeshResourceKind>(initial_mesh);

    (void)game_object.add_component<rtr::framework::component::DeformableMeshComponent>(
        runtime.resource_manager(), mesh_handle, color
    );
}

}  // namespace

int main() {
    constexpr uint32_t kWidth  = 1280;
    constexpr uint32_t kHeight = 720;

    try {
        rtr::editor::EditorAppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR IPC Fixed-End Block",
            .fixed_delta_seconds = 0.01,
            .max_fixed_steps_per_frame = 20,
        });

        auto editor_host = std::make_shared<rtr::editor::EditorHost>(runtime);
        editor_host->register_panel(std::make_unique<rtr::editor::SceneViewPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::HierarchyPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::InspectorPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::StatsPanel>());
        editor_host->register_panel(std::make_unique<rtr::editor::LoggerPanel>());

        runtime.renderer().output_backend().bind_editor_host(editor_host);
        rtr::editor::bind_input_capture_to_editor(runtime.input_system(), runtime.renderer().output_backend());
        auto editor_pipeline = std::make_unique<rtr::system::render::ForwardPipeline>(
            runtime.renderer().build_pipeline_runtime()
        );
        runtime.set_pipeline(std::move(editor_pipeline));

        auto& scene = runtime.world().create_scene("ipc_fixed_end_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.4f, 1.9f, 4.8f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(
            runtime.input_system().state()
        );
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 1.3f, 0.0f});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({2.0f, 4.0f, 4.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.95f, 0.85f});
        point_light.set_intensity(50.0f);
        point_light.set_range(20.0f);

        auto& ground_go = scene.create_game_object("ground");
        const auto ground_mesh = runtime.resource_manager()
            .create_from_relative_path<rtr::resource::MeshResourceKind>("models/colored_quad.obj");
        (void)ground_go.add_component<rtr::framework::component::StaticMeshComponent>(
            runtime.resource_manager(), ground_mesh, pbpt::math::Vec4{0.25f, 0.25f, 0.28f, 1.0f}
        );
        ground_go.node().set_local_position({0.0f, 0.0f, 0.0f});
        ground_go.node().set_local_rotation(
            pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f})
        );
        ground_go.node().set_local_scale({12.0f, 12.0f, 1.0f});

        auto& block_go = scene.create_game_object("ipc_fixed_end_block");
        register_ipc_tet_object(
            block_go,
            runtime,
            make_fixed_end_block(),
            pbpt::math::Vec4{0.75f, 0.48f, 0.3f, 1.0f}
        );
        runtime.set_callbacks(rtr::app::RuntimeCallbacks{
            .on_post_update =
                [&](rtr::app::RuntimeContext& ctx) {
                    editor_host->begin_frame(rtr::editor::EditorFrameData{
                        .frame_serial = ctx.frame_serial,
                        .delta_seconds = ctx.delta_seconds,
                        .paused = ctx.paused,
                    });

                    auto* active_scene = ctx.world.active_scene();
                    if (active_scene == nullptr) {
                        return;
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
                    if (auto* perspective =
                            dynamic_cast<rtr::framework::component::PerspectiveCamera*>(active_camera);
                        perspective != nullptr) {
                        perspective->aspect_ratio() = static_cast<float>(fb_w) / static_cast<float>(fb_h);
                    }
                },
            .on_pre_render =
                [](rtr::app::RuntimeContext& ctx) {
                    if (ctx.input_system.state().key_down(rtr::system::input::KeyCode::ESCAPE)) {
                        ctx.renderer.window().close();
                    }
                }
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
