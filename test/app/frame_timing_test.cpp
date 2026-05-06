#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rtr/app/frame_stepper.hpp"
#include "rtr/app/frame_time_policy.hpp"
#include "rtr/framework/component/component.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/physics/physics_system.hpp"

namespace rtr::app::test {

static_assert(FrameTimePolicyConcept<RealtimeFrameTimePolicy>);
static_assert(FrameTimePolicyConcept<OfflineFrameTimePolicy>);

class CountingComponent final : public framework::component::Component {
public:
    explicit CountingComponent(framework::core::GameObject& owner)
        : Component(owner) {}

    std::size_t fixed_count{0};
    std::size_t update_count{0};
    std::size_t late_count{0};
    double last_frame_delta{0.0};
    std::uint64_t last_frame_serial{0};

    void on_fixed_update(const framework::core::FixedTickContext& /*ctx*/) override {
        ++fixed_count;
    }

    void on_update(const framework::core::FrameTickContext& ctx) override {
        ++update_count;
        last_frame_delta = ctx.delta_seconds;
        last_frame_serial = ctx.frame_serial;
    }

    void on_late_update(const framework::core::FrameTickContext& /*ctx*/) override {
        ++late_count;
    }
};

TEST(FrameTimePolicyTest, RealtimeNegativeDeltaClampsToZero) {
    std::vector<double> timeline{1.0, 0.5};
    std::size_t next_time_index = 0;
    RealtimeFrameTimePolicy policy(0.01, 4, 0.1, [&]() {
        const std::size_t index = std::min(next_time_index, timeline.size() - 1);
        ++next_time_index;
        return timeline[index];
    });
    policy.reset();

    const FrameExecutionPlan plan = policy.make_plan(false);

    EXPECT_EQ(plan.fixed_steps_to_run, 0u);
    EXPECT_DOUBLE_EQ(plan.fixed_dt, 0.01);
    EXPECT_DOUBLE_EQ(plan.frame_delta_seconds, 0.0);
}

TEST(FrameTimePolicyTest, RealtimeMaxFrameDeltaClampWorks) {
    std::vector<double> timeline{0.0, 0.5};
    std::size_t next_time_index = 0;
    RealtimeFrameTimePolicy policy(0.01, 20, 0.1, [&]() {
        const std::size_t index = std::min(next_time_index, timeline.size() - 1);
        ++next_time_index;
        return timeline[index];
    });
    policy.reset();

    const FrameExecutionPlan plan = policy.make_plan(false);

    EXPECT_EQ(plan.fixed_steps_to_run, 10u);
    EXPECT_DOUBLE_EQ(plan.fixed_dt, 0.01);
    EXPECT_DOUBLE_EQ(plan.frame_delta_seconds, 0.1);
}

TEST(FrameTimePolicyTest, RealtimeAccumulatorCarriesRemainderAcrossCalls) {
    std::vector<double> timeline{0.0, 0.025, 0.031};
    std::size_t next_time_index = 0;
    RealtimeFrameTimePolicy policy(0.01, 8, 0.1, [&]() {
        const std::size_t index = std::min(next_time_index, timeline.size() - 1);
        ++next_time_index;
        return timeline[index];
    });
    policy.reset();

    const FrameExecutionPlan first = policy.make_plan(false);
    const FrameExecutionPlan second = policy.make_plan(false);

    EXPECT_EQ(first.fixed_steps_to_run, 2u);
    EXPECT_DOUBLE_EQ(first.frame_delta_seconds, 0.025);
    EXPECT_EQ(second.fixed_steps_to_run, 1u);
    EXPECT_NEAR(second.frame_delta_seconds, 0.006, 1e-12);
}

TEST(FrameTimePolicyTest, RealtimeFixedStepCapIsRespected) {
    std::vector<double> timeline{0.0, 0.5};
    std::size_t next_time_index = 0;
    RealtimeFrameTimePolicy policy(0.01, 2, 0.1, [&]() {
        const std::size_t index = std::min(next_time_index, timeline.size() - 1);
        ++next_time_index;
        return timeline[index];
    });
    policy.reset();

    const FrameExecutionPlan plan = policy.make_plan(false);

    EXPECT_EQ(plan.fixed_steps_to_run, 2u);
    EXPECT_DOUBLE_EQ(plan.frame_delta_seconds, 0.1);
}

TEST(FrameTimePolicyTest, RealtimePausedReturnsZeroPlanWithoutAccumulatingPausedTime) {
    std::vector<double> timeline{0.0, 0.5, 0.51};
    std::size_t next_time_index = 0;
    RealtimeFrameTimePolicy policy(0.01, 8, 0.1, [&]() {
        const std::size_t index = std::min(next_time_index, timeline.size() - 1);
        ++next_time_index;
        return timeline[index];
    });
    policy.reset();

    const FrameExecutionPlan paused = policy.make_plan(true);
    const FrameExecutionPlan resumed = policy.make_plan(false);

    EXPECT_EQ(paused.fixed_steps_to_run, 0u);
    EXPECT_DOUBLE_EQ(paused.fixed_dt, 0.0);
    EXPECT_DOUBLE_EQ(paused.frame_delta_seconds, 0.0);

    EXPECT_EQ(resumed.fixed_steps_to_run, 1u);
    EXPECT_DOUBLE_EQ(resumed.fixed_dt, 0.01);
    EXPECT_NEAR(resumed.frame_delta_seconds, 0.01, 1e-12);
}

TEST(FrameTimePolicyTest, OfflinePlanMatchesConfiguredSimulationStep) {
    const OfflineFrameTimePolicy policy(0.01, 3);

    const FrameExecutionPlan plan = policy.make_plan(false);

    EXPECT_EQ(plan.fixed_steps_to_run, 3u);
    EXPECT_DOUBLE_EQ(plan.fixed_dt, 0.01);
    EXPECT_DOUBLE_EQ(plan.frame_delta_seconds, 0.03);
}

TEST(FrameTimePolicyTest, OfflinePausedReturnsZeroPlan) {
    const OfflineFrameTimePolicy policy(0.01, 3);

    const FrameExecutionPlan plan = policy.make_plan(true);

    EXPECT_EQ(plan.fixed_steps_to_run, 0u);
    EXPECT_DOUBLE_EQ(plan.fixed_dt, 0.0);
    EXPECT_DOUBLE_EQ(plan.frame_delta_seconds, 0.0);
}

TEST(FrameStepperTest, ExecuteRunsFixedAndVariableUpdates) {
    resource::ResourceManager resources;
    framework::core::World world(resources);
    system::physics::PhysicsSystem physics_system;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();
    std::uint64_t fixed_tick_serial = 0;

    FrameStepperContext ctx{
        .world = world,
        .physics_system = physics_system,
        .fixed_tick_serial = fixed_tick_serial,
        .frame_serial = 7,
    };

    FrameStepper stepper;
    stepper.execute(FrameExecutionPlan{
        .fixed_steps_to_run = 2,
        .fixed_dt = 0.01,
        .frame_delta_seconds = 0.02,
    }, ctx);

    EXPECT_EQ(comp.fixed_count, 2u);
    EXPECT_EQ(comp.update_count, 1u);
    EXPECT_EQ(comp.late_count, 1u);
    EXPECT_EQ(fixed_tick_serial, 2u);
    EXPECT_DOUBLE_EQ(comp.last_frame_delta, 0.02);
    EXPECT_EQ(comp.last_frame_serial, 7u);
}

TEST(FrameStepperTest, ExecuteSkipsAllWorkForZeroPlan) {
    resource::ResourceManager resources;
    framework::core::World world(resources);
    system::physics::PhysicsSystem physics_system;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();
    std::uint64_t fixed_tick_serial = 0;

    FrameStepperContext ctx{
        .world = world,
        .physics_system = physics_system,
        .fixed_tick_serial = fixed_tick_serial,
        .frame_serial = 3,
    };

    FrameStepper stepper;
    stepper.execute(FrameExecutionPlan{}, ctx);

    EXPECT_EQ(comp.fixed_count, 0u);
    EXPECT_EQ(comp.update_count, 0u);
    EXPECT_EQ(comp.late_count, 0u);
    EXPECT_EQ(fixed_tick_serial, 0u);
}

TEST(FrameStepperTest, ExecuteStillRunsVariableUpdateForZeroDeltaNonPausedPlan) {
    resource::ResourceManager resources;
    framework::core::World world(resources);
    system::physics::PhysicsSystem physics_system;
    auto& scene = world.create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();
    std::uint64_t fixed_tick_serial = 0;

    FrameStepperContext ctx{
        .world = world,
        .physics_system = physics_system,
        .fixed_tick_serial = fixed_tick_serial,
        .frame_serial = 9,
    };

    FrameStepper stepper;
    stepper.execute(FrameExecutionPlan{
        .fixed_steps_to_run = 0,
        .fixed_dt = 0.01,
        .frame_delta_seconds = 0.0,
    }, ctx);

    EXPECT_EQ(comp.fixed_count, 0u);
    EXPECT_EQ(comp.update_count, 1u);
    EXPECT_EQ(comp.late_count, 1u);
    EXPECT_EQ(fixed_tick_serial, 0u);
    EXPECT_DOUBLE_EQ(comp.last_frame_delta, 0.0);
    EXPECT_EQ(comp.last_frame_serial, 9u);
}

} // namespace rtr::app::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
