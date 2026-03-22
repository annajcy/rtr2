#include <pbpt/math/math.h>

#include "gtest/gtest.h"

#include "rtr/framework/core/scene.hpp"
#include "rtr/framework/integration/physics/scene_physics_step.hpp"
#include "rtr/system/physics/physics_system.hpp"

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

}  // namespace rtr::framework::integration::physics::test
