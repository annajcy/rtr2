#include <pbpt/math/math.h>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <vector>

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
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/component/physics/cloth/cloth_component.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_types.hpp"
#include "rtr/utils/obj_types.hpp"

namespace {

constexpr uint32_t kWidth = 1280;
constexpr uint32_t kHeight = 720;
const rtr::system::physics::ClothParams kLab2ClothParams{
    .default_vertex_mass = 0.5f,
    .gravity = pbpt::math::Vec3{0.0f, -3.5f, 0.0f},
    .edge_stiffness = 3200.0f,
    .bend_stiffness = 35.0f,
    .spring_damping = 18.0f,
    .velocity_damping = 2.0f,
    .substeps = 48u,
};

rtr::utils::ObjMeshData make_cloth_patch(std::uint32_t columns, std::uint32_t rows,
                                         float width, float height) {
    if (columns < 2u || rows < 2u) {
        throw std::invalid_argument("Cloth patch requires at least a 2x2 grid.");
    }

    rtr::utils::ObjMeshData mesh{};
    mesh.vertices.reserve(static_cast<std::size_t>(columns) * static_cast<std::size_t>(rows));
    mesh.indices.reserve(static_cast<std::size_t>(columns - 1u) * static_cast<std::size_t>(rows - 1u) * 6u);

    for (std::uint32_t row = 0; row < rows; ++row) {
        const float v = rows > 1u ? static_cast<float>(row) / static_cast<float>(rows - 1u) : 0.0f;
        const float y = (1.0f - v) * height;
        for (std::uint32_t col = 0; col < columns; ++col) {
            const float u = columns > 1u ? static_cast<float>(col) / static_cast<float>(columns - 1u) : 0.0f;
            const float x = (u - 0.5f) * width;
            mesh.vertices.push_back(rtr::utils::ObjVertex{
                .position = pbpt::math::Vec3{x, y, 0.0f},
                .uv = pbpt::math::Vec2{u, v},
                .normal = pbpt::math::Vec3{0.0f, 0.0f, 1.0f},
            });
        }
    }

    for (std::uint32_t row = 0; row + 1u < rows; ++row) {
        for (std::uint32_t col = 0; col + 1u < columns; ++col) {
            const std::uint32_t i0 = row * columns + col;
            const std::uint32_t i1 = i0 + 1u;
            const std::uint32_t i2 = i0 + columns;
            const std::uint32_t i3 = i2 + 1u;
            mesh.indices.insert(mesh.indices.end(), {i0, i2, i1, i1, i2, i3});
        }
    }

    return mesh;
}

}  // namespace

int main() {
    try {
        rtr::app::AppRuntime runtime(rtr::app::AppRuntimeConfig{
            .window_width = kWidth,
            .window_height = kHeight,
            .window_title = "RTR GAMES103 Lab 2 - Mass-Spring Cloth",
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

        auto& scene = runtime.world().create_scene("lab2_cloth_scene");

        auto& camera_go = scene.create_game_object("main_camera");
        auto& camera = camera_go.add_component<rtr::framework::component::PerspectiveCamera>();
        camera.aspect_ratio() = static_cast<float>(kWidth) / static_cast<float>(kHeight);
        camera.set_active(true);
        camera_go.node().set_local_position({0.0f, 1.6f, 6.0f});
        camera_go.add_component<rtr::framework::component::FreeLookCameraController>(runtime.input_system().state());
        camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 1.0f, 0.0f});

        auto& light_go = scene.create_game_object("main_light");
        light_go.node().set_local_position({2.0f, 4.0f, 3.0f});
        auto& point_light = light_go.add_component<rtr::framework::component::light::PointLight>();
        point_light.set_color({1.0f, 0.97f, 0.92f});
        point_light.set_intensity(55.0f);
        point_light.set_range(30.0f);

        auto& ground_go = scene.create_game_object("ground");
        const auto ground_mesh = runtime.resource_manager().create_from_relative_path<rtr::resource::MeshResourceKind>(
            "models/colored_quad.obj");
        (void)ground_go.add_component<rtr::framework::component::StaticMeshComponent>(
            runtime.resource_manager(), ground_mesh, pbpt::math::Vec4{0.24f, 0.27f, 0.30f, 1.0f});
        ground_go.node().set_local_position({0.0f, -1.0f, 0.0f});
        ground_go.node().set_local_rotation(
            pbpt::math::angle_axis(pbpt::math::radians(-90.0f), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}));
        ground_go.node().set_local_scale({12.0f, 12.0f, 1.0f});

        const std::uint32_t cloth_columns = 26u;
        const std::uint32_t cloth_rows = 26u;
        const auto cloth_mesh_handle = runtime.resource_manager().create<rtr::resource::DeformableMeshResourceKind>(
            make_cloth_patch(cloth_columns, cloth_rows, 3.0f, 2.6f)
        );
        const std::vector<rtr::system::physics::VertexID> pinned_vertices = {
            0,
            static_cast<rtr::system::physics::VertexID>(cloth_columns - 1u),
        };

        auto& cloth_go = scene.create_game_object("cloth_patch");
        cloth_go.node().set_local_position({0.0f, 0.2f, 0.0f});
        (void)cloth_go.add_component<rtr::framework::component::DeformableMeshComponent>(
            runtime.resource_manager(), cloth_mesh_handle, pbpt::math::Vec4{0.83f, 0.59f, 0.44f, 1.0f});
        (void)cloth_go.add_component<rtr::framework::component::ClothComponent>(
            runtime.physics_system().cloth_world(),
            pinned_vertices,
            kLab2ClothParams
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
