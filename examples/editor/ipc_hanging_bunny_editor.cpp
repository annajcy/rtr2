#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>

#include <Eigen/Core>
#include <pbpt/math/math.h>

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
#include "rtr/system/physics/ipc/model/mesh_tet_converter/mesh_to_tet.hpp"
#include "rtr/utils/obj_io.hpp"

namespace {

rtr::utils::ObjMeshData load_hanging_bunny_surface() {
    rtr::utils::ObjMeshData mesh =
        rtr::utils::load_obj_from_path("assets/models/cube.obj");

    Eigen::Vector3d bbox_min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    Eigen::Vector3d bbox_max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
    for (const auto& vertex : mesh.vertices) {
        const Eigen::Vector3d position(
            static_cast<double>(vertex.position.x()),
            static_cast<double>(vertex.position.y()),
            static_cast<double>(vertex.position.z())
        );
        bbox_min = bbox_min.cwiseMin(position);
        bbox_max = bbox_max.cwiseMax(position);
    }

    constexpr double kScale = 1.0;
    const Eigen::Vector3d bbox_center = 0.5 * (bbox_min + bbox_max);
    const Eigen::Vector3d scaled_bbox_center = kScale * bbox_center;
    const double scaled_bbox_max_y = kScale * bbox_max.y();
    const Eigen::Vector3d translation(-scaled_bbox_center.x(), 1.8 - scaled_bbox_max_y, -scaled_bbox_center.z());

    for (auto& vertex : mesh.vertices) {
        const Eigen::Vector3d position(
            static_cast<double>(vertex.position.x()),
            static_cast<double>(vertex.position.y()),
            static_cast<double>(vertex.position.z())
        );
        const Eigen::Vector3d transformed = kScale * position + translation;
        vertex.position = pbpt::math::Vec3(
            static_cast<float>(transformed.x()),
            static_cast<float>(transformed.y()),
            static_cast<float>(transformed.z())
        );
    }

    return mesh;
}

rtr::system::physics::ipc::TetBody make_hanging_bunny_body() {
    namespace ipc = rtr::system::physics::ipc;

    if (!ipc::ftetwild_available()) {
        throw std::runtime_error("fTetWild is not available. Reconfigure with RTR_BUILD_FTETWILD=ON.");
    }

    ipc::TetMeshingParams params{};
    params.ideal_edge_length_rel = 0.2;
    params.eps_rel = 1e-3;
    params.max_its = 80;
    params.skip_simplify = false;
    params.quiet = true;

    ipc::TetBody body = ipc::obj_mesh_to_tet_body(
        load_hanging_bunny_surface(),
        ipc::FixedCorotatedMaterial{
            .mass_density = 1000.0,
            .youngs_modulus = 5e4,
            .poisson_ratio = 0.3,
        },
        params
    );

    double max_y = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    for (const Eigen::Vector3d& position : body.geometry.rest_positions) {
        max_y = std::max(max_y, position.y());
        min_y = std::min(min_y, position.y());
    }

    const double bbox_height = std::max(max_y - min_y, 1e-6);
    body.fixed_vertices.assign(body.vertex_count(), false);

    auto mark_fixed_vertices = [&](double band_ratio) {
        const double cutoff_y = max_y - band_ratio * bbox_height;
        std::size_t fixed_count = 0;
        std::fill(body.fixed_vertices.begin(), body.fixed_vertices.end(), false);
        for (std::size_t vertex_index = 0; vertex_index < body.vertex_count(); ++vertex_index) {
            if (body.geometry.rest_positions[vertex_index].y() >= cutoff_y) {
                body.fixed_vertices[vertex_index] = true;
                ++fixed_count;
            }
        }
        return fixed_count;
    };

    std::size_t fixed_count = mark_fixed_vertices(0.03);
    if (fixed_count < 8u) {
        fixed_count = mark_fixed_vertices(0.06);
    }
    if (fixed_count == 0u) {
        throw std::runtime_error("Failed to select fixed vertices for hanging bunny.");
    }

    return body;
}

void register_ipc_tet_object(rtr::framework::core::GameObject& game_object,
                             rtr::editor::EditorAppRuntime& runtime,
                             rtr::system::physics::ipc::TetBody body,
                             const pbpt::math::Vec4& color) {
    auto& ipc_component = game_object.add_component<rtr::framework::component::IPCTetComponent>(std::move(body));
    const rtr::utils::ObjMeshData initial_mesh = ipc_component.mesh_cache();
    const auto mesh_handle = runtime.resource_manager().create<rtr::resource::DeformableMeshResourceKind>(initial_mesh);

    (void)game_object.add_component<rtr::framework::component::DeformableMeshComponent>(
        runtime.resource_manager(),
        mesh_handle,
        color
    );
}

}  // namespace

int main() {
    constexpr uint32_t kWidth = 1280;
    constexpr uint32_t kHeight = 720;

    try {
        rtr::editor::EditorAppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR IPC Hanging Bunny",
            .fixed_delta_seconds = 0.01,
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

        auto& scene = runtime.world().create_scene("ipc_hanging_bunny_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
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

        auto& ground_go = scene.create_game_object("ground");
        const auto ground_mesh =
            runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>("models/colored_quad.obj");
        (void)ground_go.add_component<rtr::framework::component::StaticMeshComponent>(
            runtime.resource_manager(),
            ground_mesh,
            pbpt::math::Vec4{0.25f, 0.25f, 0.28f, 1.0f}
        );
        ground_go.node().set_local_position({0.0f, 0.0f, 0.0f});
        ground_go.node().set_local_rotation(
            pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f})
        );
        ground_go.node().set_local_scale({15.0f, 15.0f, 1.0f});

        auto& bunny_go = scene.create_game_object("ipc_hanging_bunny");
        register_ipc_tet_object(
            bunny_go,
            runtime,
            make_hanging_bunny_body(),
            pbpt::math::Vec4{0.90f, 0.85f, 0.78f, 1.0f}
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
