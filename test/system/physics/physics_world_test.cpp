#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/system/physics/physics_world.hpp"

namespace rtr::system::physics::test {

TEST(PhysicsWorldTest, DefaultsSolverIterationsToEight) {
    PhysicsWorld world;

    EXPECT_EQ(world.solver_iterations(), PhysicsWorld::kDefaultSolverIterations);
}

TEST(PhysicsWorldTest, SetterUpdatesSolverIterations) {
    PhysicsWorld world;

    world.set_solver_iterations(4);

    EXPECT_EQ(world.solver_iterations(), 4u);
}

TEST(PhysicsWorldTest, RejectsZeroSolverIterations) {
    PhysicsWorld world;

    EXPECT_THROW(world.set_solver_iterations(0), std::invalid_argument);
}

}  // namespace rtr::system::physics::test
