#include "gtest/gtest.h"

#include "rtr/system/physics/physics_system.hpp"

namespace rtr::system::physics::test {

TEST(PhysicsSystemTest, EmptyStepRunsAllWorldStagesOnce) {
    PhysicsSystem physics_system{};

    physics_system.step(1.0f / 60.0f);

    EXPECT_EQ(physics_system.rigid_body_system().velocity_iterations(),
              rb::RigidBodySystem::kDefaultVelocityIterations);
    EXPECT_EQ(physics_system.rigid_body_system().position_iterations(),
              rb::RigidBodySystem::kDefaultPositionIterations);
}

}  // namespace rtr::system::physics::test
