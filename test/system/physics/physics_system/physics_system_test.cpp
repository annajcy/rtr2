#include "gtest/gtest.h"

#include "rtr/framework/core/scene.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::system::physics::test {

TEST(PhysicsSystemTest, EmptyStepRunsAllWorldStagesOnce) {
    framework::core::Scene scene(1);
    PhysicsSystem          physics_system{};

    physics_system.step(scene, 1.0f / 60.0f);

    EXPECT_EQ(physics_system.rigid_body_world().velocity_iterations(),
              RigidBodyWorld::kDefaultVelocityIterations);
    EXPECT_EQ(physics_system.rigid_body_world().position_iterations(),
              RigidBodyWorld::kDefaultPositionIterations);
}

}  // namespace rtr::system::physics::test
