#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/ipc_scene_sync.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::framework::integration::physics::test {

namespace {

system::physics::ipc::TetBody make_block(const Eigen::Vector3d& origin) {
    return system::physics::ipc::generate_tet_block(1, 1, 1, 0.25, origin);
}

}  // namespace

TEST(IPCSceneSyncTest, SyncWritesBackMeshForBodyWithGlobalOffset) {
    resource::ResourceManager resources{};
    core::Scene scene(1);
    system::physics::ipc::IPCSystem ipc_system{};

    auto body0 = make_block(Eigen::Vector3d(-1.0, 1.0, 0.0));
    auto body1 = make_block(Eigen::Vector3d(1.0, 1.5, 0.0));

    const auto surface1 = system::physics::ipc::extract_tet_surface(body1);
    const auto initial_mesh1 = system::physics::ipc::tet_to_mesh(body1.geometry, surface1);
    const auto mesh_handle = resources.create<resource::DeformableMeshResourceKind>(initial_mesh1);

    auto& go = scene.create_game_object("ipc_body");
    (void)go.add_component<framework::component::DeformableMeshComponent>(
        resources, mesh_handle, pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    );

    ipc_system.add_tet_body(std::move(body0));
    ipc_system.add_tet_body(std::move(body1));
    (void)go.add_component<framework::component::IPCTetComponent>(1u, surface1, initial_mesh1);

    ipc_system.initialize();
    ipc_system.step();

    sync_ipc_to_scene(scene, ipc_system);

    const auto& cpu_mesh = resources.cpu<resource::DeformableMeshResourceKind>(mesh_handle);
    const auto& body = ipc_system.tet_body(1u);
    const std::size_t base_vertex = body.info.dof_offset / 3u;

    ASSERT_EQ(cpu_mesh.vertices.size(), surface1.surface_vertex_ids.size());
    for (std::size_t i = 0; i < surface1.surface_vertex_ids.size(); ++i) {
        const auto expected = ipc_system.state().position(base_vertex + surface1.surface_vertex_ids[i]);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.x(), static_cast<float>(expected.x()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.y(), static_cast<float>(expected.y()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.z(), static_cast<float>(expected.z()), 1e-5f);
    }
}

TEST(IPCSceneSyncTest, MissingDeformableComponentIsSkipped) {
    core::Scene scene(1);
    system::physics::ipc::IPCSystem ipc_system{};

    auto body = make_block(Eigen::Vector3d(0.0, 1.0, 0.0));
    const auto surface = system::physics::ipc::extract_tet_surface(body);
    const auto mesh_cache = system::physics::ipc::tet_to_mesh(body.geometry, surface);

    auto& go = scene.create_game_object("ipc_body");
    ipc_system.add_tet_body(std::move(body));
    (void)go.add_component<framework::component::IPCTetComponent>(0u, surface, mesh_cache);

    ipc_system.initialize();
    ipc_system.step();

    EXPECT_NO_THROW(sync_ipc_to_scene(scene, ipc_system));
}

TEST(IPCSceneSyncTest, InvalidBodyIndexThrows) {
    resource::ResourceManager resources{};
    core::Scene scene(1);
    system::physics::ipc::IPCSystem ipc_system{};

    auto body = make_block(Eigen::Vector3d(0.0, 1.0, 0.0));
    const auto surface = system::physics::ipc::extract_tet_surface(body);
    const auto mesh_cache = system::physics::ipc::tet_to_mesh(body.geometry, surface);
    const auto mesh_handle = resources.create<resource::DeformableMeshResourceKind>(mesh_cache);

    auto& go = scene.create_game_object("ipc_body");
    (void)go.add_component<framework::component::DeformableMeshComponent>(
        resources, mesh_handle, pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    );
    ipc_system.add_tet_body(std::move(body));
    (void)go.add_component<framework::component::IPCTetComponent>(1u, surface, mesh_cache);

    ipc_system.initialize();

    EXPECT_THROW(sync_ipc_to_scene(scene, ipc_system), std::out_of_range);
}

}  // namespace rtr::framework::integration::physics::test
