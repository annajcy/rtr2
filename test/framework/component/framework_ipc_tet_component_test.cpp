#include "gtest/gtest.h"

#include <Eigen/Core>

#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/ipc_scene_sync.hpp"
#include "rtr/system/physics/ipc/core/ipc_system.hpp"

namespace rtr::framework::component::test {
namespace {

system::physics::ipc::TetBody make_single_tet_body() {
    using namespace rtr::system::physics::ipc;

    TetBody body{};
    body.geometry.rest_positions = {
        Eigen::Vector3d(0.0, 0.0, 0.0),
        Eigen::Vector3d(1.0, 0.0, 0.0),
        Eigen::Vector3d(0.0, 1.0, 0.0),
        Eigen::Vector3d(0.0, 0.0, 1.0),
    };
    body.geometry.tets = {{{0, 1, 2, 3}}};
    body.material = system::physics::ipc::FixedCorotatedMaterial{
        .mass_density = 2.0,
        .youngs_modulus = 100.0,
        .poisson_ratio = 0.3,
    };
    return body;
}

}  // namespace

TEST(FrameworkIPCTetComponentTest, EnableRegistersBodyAndDisableRemovesIt) {
    system::physics::ipc::IPCSystem ipc_world{};
    core::Scene scene(1);
    auto& go = scene.create_game_object("ipc_body");

    auto& ipc_tet = go.add_component<IPCTetComponent>(make_single_tet_body());
    integration::physics::sync_scene_to_ipc(scene, ipc_world);

    const auto initial_body_id = ipc_world.try_get_tet_body_id_for_owner(go.id());
    ASSERT_TRUE(initial_body_id.has_value());
    EXPECT_TRUE(ipc_world.has_tet_body(*initial_body_id));

    go.set_component_enabled<IPCTetComponent>(false);
    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    EXPECT_FALSE(ipc_world.has_tet_body_for_owner(go.id()));
    EXPECT_FALSE(ipc_world.has_tet_body(*initial_body_id));

    go.set_component_enabled<IPCTetComponent>(true);
    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    const auto next_body_id = ipc_world.try_get_tet_body_id_for_owner(go.id());
    ASSERT_TRUE(next_body_id.has_value());
    EXPECT_NE(*next_body_id, *initial_body_id);
    EXPECT_TRUE(ipc_world.has_tet_body(*next_body_id));
}

TEST(FrameworkIPCTetComponentTest, ApplyingSourceMaterialUpdatesRuntimeBody) {
    system::physics::ipc::IPCSystem ipc_world{};
    core::Scene scene(1);
    auto& go = scene.create_game_object("ipc_body");

    auto& ipc_tet = go.add_component<IPCTetComponent>(make_single_tet_body());
    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    ipc_world.step(0.01);

    ASSERT_TRUE(ipc_tet.set_density(4.0));

    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    const auto* runtime_body = ipc_world.try_get_tet_body_for_owner(go.id());
    ASSERT_NE(runtime_body, nullptr);
    const auto& runtime_material = std::get<system::physics::ipc::FixedCorotatedMaterial>(runtime_body->material);

    EXPECT_DOUBLE_EQ(runtime_material.mass_density, 4.0);
    EXPECT_GT(ipc_world.state().mass_diag.minCoeff(), 0.0);
    EXPECT_NEAR(ipc_world.state().mass_diag[0], 4.0 / 2.0 * (1.0 / 12.0), 1e-12);
}

TEST(FrameworkIPCTetComponentTest, ApplyingSourceMaterialWhileDisabledOnlyUpdatesSource) {
    system::physics::ipc::IPCSystem ipc_world{};
    core::Scene scene(1);
    auto& go = scene.create_game_object("ipc_body");

    auto& ipc_tet = go.add_component<IPCTetComponent>(make_single_tet_body());
    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    const auto first_body_id = ipc_world.try_get_tet_body_id_for_owner(go.id());
    ASSERT_TRUE(first_body_id.has_value());

    go.set_component_enabled<IPCTetComponent>(false);
    integration::physics::sync_scene_to_ipc(scene, ipc_world);

    ASSERT_TRUE(ipc_tet.set_youngs_modulus(4321.0));

    EXPECT_TRUE(ipc_tet.source_dirty());
    EXPECT_FALSE(ipc_world.has_tet_body(*first_body_id));

    go.set_component_enabled<IPCTetComponent>(true);
    integration::physics::sync_scene_to_ipc(scene, ipc_world);
    const auto* runtime_body = ipc_world.try_get_tet_body_for_owner(go.id());
    ASSERT_NE(runtime_body, nullptr);
    const auto& runtime_material = std::get<system::physics::ipc::FixedCorotatedMaterial>(runtime_body->material);
    EXPECT_DOUBLE_EQ(runtime_material.youngs_modulus, 4321.0);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
