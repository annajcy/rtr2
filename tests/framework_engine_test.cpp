#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "framework/framework.hpp"

namespace rtr::framework::core::test {

class CountingComponent final : public component::Component {
public:
    std::size_t fixed_count{0};
    std::size_t update_count{0};
    std::size_t late_count{0};
    std::vector<std::string> events{};

    void on_fixed_update(const FixedTickContext& /*ctx*/) override {
        ++fixed_count;
        events.emplace_back("fixed");
    }

    void on_update(const FrameTickContext& /*ctx*/) override {
        ++update_count;
        events.emplace_back("update");
    }

    void on_late_update(const FrameTickContext& /*ctx*/) override {
        ++late_count;
        events.emplace_back("late");
    }
};

TEST(FrameworkEngineTest, RunDrivesWorldSceneGameObjectTickChain) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.01,
        .max_fixed_steps_per_frame = 8,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    std::vector<double> timeline{0.0, 0.005, 0.030, 0.041};
    std::size_t now_index = 0;
    std::size_t rendered_frames = 0;
    std::vector<std::string> loop_events{};

    engine.set_loop_hooks(Engine::LoopHooks{
        .input_begin = [&loop_events]() { loop_events.emplace_back("input_begin"); },
        .input_poll = [&loop_events]() { loop_events.emplace_back("input_poll"); },
        .input_end = [&loop_events]() { loop_events.emplace_back("input_end"); },
        .render = [&rendered_frames, &loop_events]() {
            ++rendered_frames;
            loop_events.emplace_back("render");
        },
        .should_close = [&rendered_frames]() { return rendered_frames >= 3; },
        .now_seconds = [&timeline, &now_index]() {
            const std::size_t index = std::min(now_index, timeline.size() - 1);
            ++now_index;
            return timeline[index];
        },
    });

    engine.run();

    EXPECT_EQ(rendered_frames, 3u);
    EXPECT_EQ(comp.fixed_count, 4u);
    EXPECT_EQ(comp.update_count, 3u);
    EXPECT_EQ(comp.late_count, 3u);
    EXPECT_EQ(engine.fixed_tick_index(), 4u);
    EXPECT_EQ(engine.frame_index(), 3u);

    ASSERT_EQ(loop_events.size(), 12u);
    EXPECT_EQ(loop_events[0], "input_begin");
    EXPECT_EQ(loop_events[1], "input_poll");
    EXPECT_EQ(loop_events[2], "render");
    EXPECT_EQ(loop_events[3], "input_end");
}

TEST(FrameworkEngineTest, RunRespectsMaxFixedStepsPerFrame) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.01,
        .max_fixed_steps_per_frame = 2,
        .max_frame_delta_seconds = 0.1,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    std::vector<double> timeline{0.0, 0.5};
    std::size_t now_index = 0;
    std::size_t rendered_frames = 0;

    engine.set_loop_hooks(Engine::LoopHooks{
        .render = [&rendered_frames]() { ++rendered_frames; },
        .should_close = [&rendered_frames]() { return rendered_frames >= 1; },
        .now_seconds = [&timeline, &now_index]() {
            const std::size_t index = std::min(now_index, timeline.size() - 1);
            ++now_index;
            return timeline[index];
        },
    });

    engine.run();

    EXPECT_EQ(rendered_frames, 1u);
    EXPECT_EQ(comp.fixed_count, 2u);
    EXPECT_EQ(comp.update_count, 1u);
    EXPECT_EQ(comp.late_count, 1u);
}

TEST(FrameworkEngineTest, RunCanStopViaRequestStop) {
    Engine engine(EngineConfig{
        .fixed_delta_seconds = 0.01,
        .max_fixed_steps_per_frame = 4,
    });

    auto& scene = engine.world().create_scene("main");
    auto& go = scene.create_game_object("player");
    auto& comp = go.add_component<CountingComponent>();

    std::vector<double> timeline{0.0, 0.016, 0.032};
    std::size_t now_index = 0;
    std::size_t rendered_frames = 0;

    engine.set_loop_hooks(Engine::LoopHooks{
        .render = [&]() {
            ++rendered_frames;
            engine.request_stop();
        },
        .should_close = []() { return false; },
        .now_seconds = [&timeline, &now_index]() {
            const std::size_t index = std::min(now_index, timeline.size() - 1);
            ++now_index;
            return timeline[index];
        },
    });

    engine.run();

    EXPECT_EQ(rendered_frames, 1u);
    EXPECT_TRUE(engine.stop_requested());
    EXPECT_EQ(comp.update_count, 1u);
    EXPECT_EQ(comp.late_count, 1u);
}

} // namespace rtr::framework::core::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
