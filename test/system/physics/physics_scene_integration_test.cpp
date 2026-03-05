#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/component/physics/rigid_body_component.hpp"
#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/core/tick_context.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::system::physics::test {

TEST(PhysicsSceneIntegrationTest, FixedTickIntegratesVelocityAndSyncsBackToSceneGraph) {
    PhysicsSystem physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    moving.node().set_local_position(pbpt::math::Vec3{0.0f, 0.0f, 0.0f});
    (void)moving.add_component<framework::component::RigidBodyComponent>(
        physics_system.world(), pbpt::math::Vec3{1.0f, 0.0f, 0.0f}
    );

    for (std::uint64_t i = 0; i < 10; ++i) {
        physics_system.fixed_tick(scene, framework::core::FixedTickContext{
                                            .fixed_delta_seconds = 0.1,
                                            .fixed_tick_index    = i,
                                        });
    }

    const auto pos = scene.scene_graph().node(moving.id()).local_position();
    EXPECT_NEAR(pos.x(), 1.0f, 1e-4f);
    EXPECT_NEAR(pos.y(), 0.0f, 1e-4f);
    EXPECT_NEAR(pos.z(), 0.0f, 1e-4f);
}

TEST(PhysicsSceneIntegrationTest, DestroyRemovesRigidBodyFromPhysicsWorld) {
    PhysicsSystem physics_system;
    framework::core::Scene scene(1);

    auto& moving = scene.create_game_object("moving");
    auto& rigid_body_component = moving.add_component<framework::component::RigidBodyComponent>(
        physics_system.world(), pbpt::math::Vec3{0.0f, 0.0f, 0.0f}
    );
    const auto rigid_body_id = rigid_body_component.rigid_body_id();

    EXPECT_TRUE(physics_system.world().has_rigid_body(rigid_body_id));
    EXPECT_TRUE(scene.destroy_game_object(moving.id()));
    EXPECT_FALSE(physics_system.world().has_rigid_body(rigid_body_id));
}

}  // namespace rtr::system::physics::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
