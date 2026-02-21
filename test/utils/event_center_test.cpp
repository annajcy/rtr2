#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rtr/utils/event_center.hpp"

namespace rtr::utils::test {

namespace {

struct TickEvent {
    int value{0};
};

struct ResizeEvent {
    int width{0};
    int height{0};
};

} // namespace

TEST(EventTest, SubscribePublishAndTokenReset) {
    Event<int> event;
    std::vector<int> observed;

    auto keep = event.subscribe([&observed](int value) { observed.push_back(value + 1); });
    auto remove = event.subscribe([&observed](int value) { observed.push_back(value + 100); });

    remove.reset();

    event.publish(1);
    ASSERT_EQ(observed.size(), 1u);
    EXPECT_EQ(observed[0], 2);
    EXPECT_TRUE(keep.valid());
}

TEST(EventTest, SubscriptionAutoRemovedWhenTokenDestroyed) {
    Event<int> event;
    EXPECT_EQ(event.size(), 0u);

    {
        auto token = event.subscribe([](int) {});
        EXPECT_EQ(event.size(), 1u);
        EXPECT_TRUE(token.valid());
    }

    EXPECT_EQ(event.size(), 0u);
}

TEST(EventTest, SubscribeDuringPublishAppliesNextRound) {
    Event<int> event;
    int dynamic_hits = 0;
    SubscriptionToken dynamic{};

    auto root = event.subscribe([&](int) {
        if (!dynamic.valid()) {
            dynamic = event.subscribe([&](int) { ++dynamic_hits; });
        }
    });
    (void)root;

    event.publish(1);
    EXPECT_EQ(dynamic_hits, 0);

    event.publish(1);
    EXPECT_EQ(dynamic_hits, 1);
}

TEST(EventTest, UnsubscribeDuringPublishIsSafe) {
    Event<int> event;
    int hits = 0;
    SubscriptionToken token{};
    token = event.subscribe([&](int) {
        ++hits;
        token.reset();
    });

    event.publish(1);
    EXPECT_EQ(hits, 1);
    EXPECT_EQ(event.size(), 0u);
}

TEST(EventTest, AggregatesAllCallbackExceptions) {
    Event<int> event;
    int completion_marker = 0;

    auto a = event.subscribe([](int) { throw std::runtime_error("a"); });
    auto b = event.subscribe([](int) { throw std::logic_error("b"); });
    auto c = event.subscribe([&completion_marker](int) { ++completion_marker; });
    (void)a;
    (void)b;
    (void)c;

    try {
        event.publish(7);
        FAIL() << "Expected EventDispatchException";
    } catch (const EventDispatchException& e) {
        EXPECT_EQ(e.exceptions().size(), 2u);
    }

    EXPECT_EQ(completion_marker, 1);
}

TEST(EventTest, TokenResetAfterEventDestroyedIsSafe) {
    SubscriptionToken token{};

    {
        Event<int> event;
        token = event.subscribe([](int) {});
        EXPECT_TRUE(token.valid());
    }

    EXPECT_NO_THROW(token.reset());
}

TEST(TypedEventCenterTest, TypeRoutedSubscribePublishAndClear) {
    TypedEventCenter center;

    int tick_total = 0;
    int resize_total = 0;
    auto tick = center.subscribe<TickEvent>([&tick_total](const TickEvent& e) {
        tick_total += e.value;
    });
    auto resize = center.subscribe<ResizeEvent>([&resize_total](const ResizeEvent& e) {
        resize_total += e.width + e.height;
    });
    (void)tick;
    (void)resize;

    center.publish(TickEvent{3});
    center.publish(ResizeEvent{5, 7});

    EXPECT_EQ(tick_total, 3);
    EXPECT_EQ(resize_total, 12);
    EXPECT_EQ(center.action_count<TickEvent>(), 1u);
    EXPECT_EQ(center.action_count<ResizeEvent>(), 1u);

    center.clear();
    EXPECT_EQ(center.action_count<TickEvent>(), 0u);
    EXPECT_EQ(center.action_count<ResizeEvent>(), 0u);
}

} // namespace rtr::utils::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
