#include <cstddef>

#include "gtest/gtest.h"

#include "framework/framework.hpp"

namespace rtr::framework::core::test {

class CountingComponent final : public component::Component {
public:
    std::size_t fixed_count{0};
    std::size_t update_count{0};
    std::size_t late_count{0};

    void on_fixed_update(const FixedTickContext& /*ctx*/) override {
        ++fixed_count;
    }

    void on_update(const FrameTickContext& /*ctx*/) override {
        ++update_count;
    }

    void on_late_update(const FrameTickContext& /*ctx*/) override {
        ++late_count;
    }
};

TEST(FrameworkEngineTest, RunFrameDrivesTickPhasesWithFixedAccumulator) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.01,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    engine.run_frame(0.005);
    engine.run_frame(0.025);
    engine.run_frame(0.011);

    EXPECT_EQ(comp.fixed_count, 4u);
    EXPECT_EQ(comp.update_count, 3u);
    EXPECT_EQ(comp.late_count, 3u);
    EXPECT_EQ(engine.fixed_tick_index(), 4u);
    EXPECT_EQ(engine.frame_index(), 3u);
    EXPECT_NEAR(engine.fixed_accumulator(), 0.001, 1e-9);
}

TEST(FrameworkEngineTest, RunFrameCapsFixedStepsPerFrame) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.01,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    engine.run_frame(1.0);

    EXPECT_EQ(comp.fixed_count, 4u);
    EXPECT_EQ(comp.update_count, 1u);
    EXPECT_EQ(comp.late_count, 1u);
}

TEST(FrameworkEngineTest, RunForFramesStopsAfterRequestStop) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.02,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    engine.request_stop();
    engine.run_for_frames(10);

    EXPECT_TRUE(engine.stop_requested());
    EXPECT_EQ(comp.fixed_count, 0u);
    EXPECT_EQ(comp.update_count, 0u);
    EXPECT_EQ(comp.late_count, 0u);

    engine.reset_stop_request();
    engine.run_for_frames(3);

    EXPECT_EQ(comp.fixed_count, 3u);
    EXPECT_EQ(comp.update_count, 3u);
    EXPECT_EQ(comp.late_count, 3u);
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
