#include "gtest/gtest.h"

#include <Eigen/Core>

#include "rtr/framework/component/physics/ipc/ipc_tet_component.hpp"
#include "rtr/framework/core/scene.hpp"
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

    auto& ipc_tet = go.add_component<IPCTetComponent>(ipc_world, make_single_tet_body());

    ASSERT_TRUE(ipc_tet.has_registered_body());
    const auto initial_body_id = ipc_tet.body_id();
    EXPECT_TRUE(ipc_world.has_tet_body(initial_body_id));

    go.set_component_enabled<IPCTetComponent>(false);
    EXPECT_FALSE(ipc_tet.has_registered_body());
    EXPECT_FALSE(ipc_world.has_tet_body(initial_body_id));

    go.set_component_enabled<IPCTetComponent>(true);
    EXPECT_TRUE(ipc_tet.has_registered_body());
    EXPECT_NE(ipc_tet.body_id(), initial_body_id);
    EXPECT_TRUE(ipc_world.has_tet_body(ipc_tet.body_id()));
}

TEST(FrameworkIPCTetComponentTest, ApplyingSourceMaterialUpdatesRuntimeBody) {
    system::physics::ipc::IPCSystem ipc_world{};
    core::Scene scene(1);
    auto& go = scene.create_game_object("ipc_body");

    auto& ipc_tet = go.add_component<IPCTetComponent>(ipc_world, make_single_tet_body());
    ipc_world.initialize();

    auto* source_material = ipc_tet.source_material_if<system::physics::ipc::FixedCorotatedMaterial>();
    ASSERT_NE(source_material, nullptr);
    source_material->mass_density = 4.0;

    ASSERT_TRUE(ipc_tet.apply_source_material_to_runtime());
    const auto& runtime_body = ipc_world.get_tet_body(ipc_tet.body_id());
    const auto& runtime_material = std::get<system::physics::ipc::FixedCorotatedMaterial>(runtime_body.material);

    EXPECT_DOUBLE_EQ(runtime_material.mass_density, 4.0);
    EXPECT_GT(ipc_world.state().mass_diag.minCoeff(), 0.0);
    EXPECT_NEAR(ipc_world.state().mass_diag[0], 4.0 / 2.0 * (1.0 / 12.0), 1e-12);
}

TEST(FrameworkIPCTetComponentTest, ApplyingSourceMaterialWhileDisabledOnlyUpdatesSource) {
    system::physics::ipc::IPCSystem ipc_world{};
    core::Scene scene(1);
    auto& go = scene.create_game_object("ipc_body");

    auto& ipc_tet = go.add_component<IPCTetComponent>(ipc_world, make_single_tet_body());
    const auto first_body_id = ipc_tet.body_id();

    go.set_component_enabled<IPCTetComponent>(false);

    auto* source_material = ipc_tet.source_material_if<system::physics::ipc::FixedCorotatedMaterial>();
    ASSERT_NE(source_material, nullptr);
    source_material->youngs_modulus = 4321.0;

    EXPECT_FALSE(ipc_tet.apply_source_material_to_runtime());
    EXPECT_FALSE(ipc_world.has_tet_body(first_body_id));

    go.set_component_enabled<IPCTetComponent>(true);
    const auto& runtime_body = ipc_world.get_tet_body(ipc_tet.body_id());
    const auto& runtime_material = std::get<system::physics::ipc::FixedCorotatedMaterial>(runtime_body.material);
    EXPECT_DOUBLE_EQ(runtime_material.youngs_modulus, 4321.0);
}

}  // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
