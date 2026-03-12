#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <vector>

#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#define private public
#include "rtr/system/physics/rigid_body/rigid_body_world.hpp"
#undef private

namespace rtr::system::physics::test {

TEST(RigidBodyWorldTest, DefaultsIterationsToVelocityEightAndPositionThree) {
    RigidBodyWorld world;

    EXPECT_EQ(world.velocity_iterations(), RigidBodyWorld::kDefaultVelocityIterations);
    EXPECT_EQ(world.position_iterations(), RigidBodyWorld::kDefaultPositionIterations);
}

TEST(RigidBodyWorldTest, SettersUpdateIterations) {
    RigidBodyWorld world;

    world.set_velocity_iterations(4);
    world.set_position_iterations(2);

    EXPECT_EQ(world.velocity_iterations(), 4u);
    EXPECT_EQ(world.position_iterations(), 2u);
}

TEST(RigidBodyWorldTest, RejectsZeroVelocityIterations) {
    RigidBodyWorld world;

    EXPECT_THROW(world.set_velocity_iterations(0), std::invalid_argument);
}

TEST(RigidBodyWorldTest, RejectsZeroPositionIterations) {
    RigidBodyWorld world;

    EXPECT_THROW(world.set_position_iterations(0), std::invalid_argument);
}

TEST(RigidBodyWorldTest, NormalAccumulatedImpulseCanRollbackWithoutGoingNegative) {
    RigidBodyWorld world;

    RigidBody body_a{};
    body_a.set_type(RigidBodyType::Dynamic);
    body_a.set_awake(true);
    body_a.set_use_gravity(false);
    body_a.set_restitution(0.5f);
    body_a.state().mass = 1.0f;
    body_a.state().translation.position = pbpt::math::Vec3{0.0f, 0.0f, 0.0f};
    body_a.state().translation.linear_velocity = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};

    RigidBody body_b{};
    body_b.set_type(RigidBodyType::Dynamic);
    body_b.set_awake(true);
    body_b.set_use_gravity(false);
    body_b.set_restitution(0.5f);
    body_b.state().mass = 1.0f;
    body_b.state().translation.position = pbpt::math::Vec3{0.8f, 0.0f, 0.0f};

    const auto body_a_id = world.create_rigid_body(body_a);
    const auto body_b_id = world.create_rigid_body(body_b);
    (void)world.create_collider(body_a_id, Collider{.shape = SphereShape{.radius = 0.5f}});
    (void)world.create_collider(body_b_id, Collider{.shape = SphereShape{.radius = 0.5f}});

    const auto contacts = world.collect_contacts();
    auto solver_contacts = world.build_solver_contacts(contacts);

    ASSERT_EQ(solver_contacts.size(), 1u);

    auto& contact = solver_contacts.front();
    world.apply_velocity_impulses(contact);
    const auto impulse_sum_after_first_iteration = contact.normal_impulse_sum;
    world.get_rigid_body(body_b_id).state().translation.linear_velocity = pbpt::math::Vec3{2.0f, 0.0f, 0.0f};
    world.apply_velocity_impulses(contact);

    EXPECT_GT(impulse_sum_after_first_iteration, 0.0f);
    EXPECT_LT(contact.normal_impulse_sum, impulse_sum_after_first_iteration);
    EXPECT_GE(contact.normal_impulse_sum, 0.0f);
}

}  // namespace rtr::system::physics::test
