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
    system::physics::ipc::IPCSystem ipc_system{};
    resource::ResourceManager resources{};
    core::Scene scene(1);

    auto body0 = make_block(Eigen::Vector3d(-1.0, 1.0, 0.0));
    auto body1 = make_block(Eigen::Vector3d(1.0, 1.5, 0.0));
    const auto body0_id = ipc_system.create_tet_body(std::move(body0));
    ASSERT_TRUE(ipc_system.has_tet_body(body0_id));

    auto& go = scene.create_game_object("ipc_body");
    auto& ipc_tet = go.add_component<framework::component::IPCTetComponent>(std::move(body1));
    const auto mesh_handle = resources.create<resource::DeformableMeshResourceKind>(ipc_tet.mesh_cache());
    (void)go.add_component<framework::component::DeformableMeshComponent>(
        resources, mesh_handle, pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    );
    sync_scene_to_ipc(scene, ipc_system);
    ipc_system.step(0.01);

    sync_ipc_to_scene(scene, ipc_system);

    const auto& cpu_mesh = resources.cpu<resource::DeformableMeshResourceKind>(mesh_handle);
    const auto* body = ipc_system.try_get_tet_body_for_owner(go.id());
    ASSERT_NE(body, nullptr);
    const std::size_t base_vertex = body->info.dof_offset / 3u;

    ASSERT_EQ(cpu_mesh.vertices.size(), ipc_tet.surface_cache().surface_vertex_ids.size());
    for (std::size_t i = 0; i < ipc_tet.surface_cache().surface_vertex_ids.size(); ++i) {
        const auto expected = ipc_system.state().position(base_vertex + ipc_tet.surface_cache().surface_vertex_ids[i]);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.x(), static_cast<float>(expected.x()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.y(), static_cast<float>(expected.y()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.z(), static_cast<float>(expected.z()), 1e-5f);
    }
}

TEST(IPCSceneSyncTest, MissingDeformableComponentIsSkipped) {
    system::physics::ipc::IPCSystem ipc_system{};
    core::Scene scene(1);

    auto body = make_block(Eigen::Vector3d(0.0, 1.0, 0.0));
    auto& go = scene.create_game_object("ipc_body");
    (void)go.add_component<framework::component::IPCTetComponent>(std::move(body));

    sync_scene_to_ipc(scene, ipc_system);
    ipc_system.step(0.01);

    EXPECT_NO_THROW(sync_ipc_to_scene(scene, ipc_system));
}

TEST(IPCSceneSyncTest, UnregisteredComponentIsSkipped) {
    system::physics::ipc::IPCSystem ipc_system{};
    resource::ResourceManager resources{};
    core::Scene scene(1);

    auto body = make_block(Eigen::Vector3d(0.0, 1.0, 0.0));
    auto& go = scene.create_game_object("ipc_body");
    auto& ipc_tet = go.add_component<framework::component::IPCTetComponent>(std::move(body));
    const auto mesh_handle = resources.create<resource::DeformableMeshResourceKind>(ipc_tet.mesh_cache());
    (void)go.add_component<framework::component::DeformableMeshComponent>(
        resources, mesh_handle, pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    );
    go.set_component_enabled<framework::component::IPCTetComponent>(false);
    sync_scene_to_ipc(scene, ipc_system);

    EXPECT_NO_THROW(sync_ipc_to_scene(scene, ipc_system));
}

}  // namespace rtr::framework::integration::physics::test
