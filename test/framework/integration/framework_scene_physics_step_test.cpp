#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/material/deformable_mesh_component.hpp"
#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/scene_physics_step.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/physics_system.hpp"
#include "rtr/system/physics/ipc/model/tet_body.hpp"
#include "rtr/system/physics/ipc/model/tet_mesh_convert.hpp"

namespace rtr::framework::integration::physics::test {

TEST(ScenePhysicsStepTest, HelperRunsPhysicsSystemWithoutClothSynchronization) {
    rtr::system::physics::PhysicsSystem physics_system{};

    rtr::framework::core::Scene scene(1);
    auto& go = scene.create_game_object("rigid_only");
    go.node().set_local_position(pbpt::math::Vec3{5.0f, 0.0f, 0.0f});

    step_scene_physics(scene, physics_system, 1.0f / 60.0f);

    const auto position = scene.scene_graph().node(go.id()).local_position();
    EXPECT_EQ(position, pbpt::math::Vec3(5.0f, 0.0f, 0.0f));
}

TEST(ScenePhysicsStepTest, HelperStepsAndSyncsIPCDeformableMesh) {
    resource::ResourceManager resources{};
    rtr::system::physics::PhysicsSystem physics_system{};
    rtr::framework::core::Scene scene(1);

    auto body = rtr::system::physics::ipc::generate_tet_block(1, 1, 1, 0.25, Eigen::Vector3d(0.0, 1.0, 0.0));

    auto& go = scene.create_game_object("ipc_body");
    auto& ipc_tet = go.add_component<rtr::framework::component::IPCTetComponent>(
        physics_system.ipc_system(),
        std::move(body)
    );
    const auto mesh_handle = resources.create<rtr::resource::DeformableMeshResourceKind>(ipc_tet.mesh_cache());
    (void)go.add_component<rtr::framework::component::DeformableMeshComponent>(
        resources, mesh_handle, pbpt::math::Vec4{1.0f, 1.0f, 1.0f, 1.0f}
    );

    step_scene_physics(scene, physics_system, 1.0f / 60.0f);

    const auto& cpu_mesh = resources.cpu<rtr::resource::DeformableMeshResourceKind>(mesh_handle);
    const auto& ipc_system = physics_system.ipc_system();
    ASSERT_EQ(cpu_mesh.vertices.size(), ipc_tet.surface_cache().surface_vertex_ids.size());
    for (std::size_t i = 0; i < ipc_tet.surface_cache().surface_vertex_ids.size(); ++i) {
        const auto expected = ipc_system.state().position(ipc_tet.surface_cache().surface_vertex_ids[i]);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.x(), static_cast<float>(expected.x()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.y(), static_cast<float>(expected.y()), 1e-5f);
        EXPECT_NEAR(cpu_mesh.vertices[i].position.z(), static_cast<float>(expected.z()), 1e-5f);
    }
}

}  // namespace rtr::framework::integration::physics::test
