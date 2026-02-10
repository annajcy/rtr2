#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "utils/event_center.hpp"

namespace rtr::utils::test {

TEST(EventTest, AddExecuteRemoveByHandle) {
    Event<int> event;
    std::vector<int> observed;

    const auto keep = event.add([&observed](int value) { observed.push_back(value + 1); });
    const auto remove = event.add([&observed](int value) { observed.push_back(value + 100); });

    EXPECT_TRUE(event.remove(remove));
    EXPECT_FALSE(event.remove(remove));

    event.execute(1);
    ASSERT_EQ(observed.size(), 1u);
    EXPECT_EQ(observed[0], 2);
    EXPECT_NE(keep, 0u);
}

TEST(EventCenterTest, RegisterAddTriggerAndRemoveAction) {
    EventCenter<int> center;
    center.register_event("tick");

    int total = 0;
    const auto h1 = center.add_action("tick", [&total](int v) { total += v; });
    const auto h2 = center.add_action("tick", [&total](int v) { total += v * 2; });
    EXPECT_NE(h1, 0u);
    EXPECT_NE(h2, 0u);

    center.trigger_event("tick", 3);
    EXPECT_EQ(total, 9);

    EXPECT_TRUE(center.remove_action("tick", h2));
    EXPECT_FALSE(center.remove_action("tick", h2));

    center.trigger_event("tick", 2);
    EXPECT_EQ(total, 11);
}

TEST(EventCenterTest, ThrowsWhenEventMissing) {
    EventCenter<> center;

    EXPECT_THROW(center.add_action("missing", []() {}), std::runtime_error);
    EXPECT_THROW(center.trigger_event("missing"), std::runtime_error);
    EXPECT_THROW(center.remove_action("missing", 1), std::runtime_error);
}

TEST(EventCenterTest, ClearRemovesAllEvents) {
    EventCenter<> center;
    center.register_event("ui");
    center.add_action("ui", []() {});
    EXPECT_TRUE(center.has_event("ui"));
    EXPECT_EQ(center.action_count("ui"), 1u);

    center.clear();
    EXPECT_FALSE(center.has_event("ui"));
}

} // namespace

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
