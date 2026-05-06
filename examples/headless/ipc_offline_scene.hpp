#pragma once

#include <limits>
#include <utility>

#include <pbpt/math/math.h>

#include "rtr/app/offline_runtime.hpp"
#include "rtr/framework/component/camera/camera.hpp"
#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/material/static_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"

namespace rtr::examples::headless {

inline system::physics::ipc::TetBody make_fixed_end_block() {
    namespace ipc = system::physics::ipc;

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

inline void register_ipc_tet_object(
    framework::core::GameObject& game_object,
    app::OfflineRuntime& runtime,
    system::physics::ipc::TetBody body,
    const pbpt::math::Vec4& color
) {
    auto& ipc_component = game_object.add_component<framework::component::IPCTetComponent>(std::move(body));
    const utils::ObjMeshData initial_mesh = ipc_component.mesh_cache();
    const auto mesh_handle = runtime.resource_manager()
        .create<resource::DeformableMeshResourceKind>(initial_mesh);

    (void)game_object.add_component<framework::component::DeformableMeshComponent>(
        runtime.resource_manager(),
        mesh_handle,
        color
    );
}

inline void setup_ipc_fixed_end_scene(
    app::OfflineRuntime& runtime,
    std::uint32_t width,
    std::uint32_t height
) {
    auto& scene = runtime.world().create_scene("ipc_fixed_end_scene");

    auto& camera_go = scene.create_game_object("main_camera");
    auto& camera = camera_go.add_component<framework::component::PerspectiveCamera>();
    camera.aspect_ratio() = static_cast<float>(width) / static_cast<float>(height);
    camera.set_active(true);
    camera_go.node().set_local_position({0.4f, 1.9f, 4.8f});
    camera.camera_look_at_point_world(pbpt::math::Vec3{0.0f, 1.3f, 0.0f});

    auto& light_go = scene.create_game_object("main_light");
    light_go.node().set_local_position({2.0f, 4.0f, 4.0f});
    auto& point_light = light_go.add_component<framework::component::light::PointLight>();
    point_light.set_color({1.0f, 0.95f, 0.85f});
    point_light.set_intensity(50.0f);
    point_light.set_range(20.0f);

    auto& ground_go = scene.create_game_object("ground");
    const auto ground_mesh = runtime.resource_manager()
        .create_from_relative_path<resource::MeshResourceKind>("models/colored_quad.obj");
    (void)ground_go.add_component<framework::component::StaticMeshComponent>(
        runtime.resource_manager(),
        ground_mesh,
        pbpt::math::Vec4{0.25f, 0.25f, 0.28f, 1.0f}
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
}

}  // namespace rtr::examples::headless
