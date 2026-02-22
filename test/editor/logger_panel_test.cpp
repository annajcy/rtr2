#include <memory>

#include "gtest/gtest.h"

#include "rtr/editor/panel/logger_panel.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::editor::test {

namespace {

void init_test_logging() {
    utils::shutdown_logging();
    utils::LogConfig config{};
    config.enable_console = false;
    config.enable_file = false;
    config.level = utils::LogLevel::debug;
    utils::init_logging(config);
}

} // namespace

TEST(LoggerPanelTest, HasStableIdAndOrder) {
    init_test_logging();
    LoggerPanel panel;

    EXPECT_EQ(panel.id(), "logger");
    EXPECT_EQ(panel.order(), 350);

    utils::shutdown_logging();
}

TEST(LoggerPanelTest, ReceivesRealtimeLogsIntoLocalCache) {
    init_test_logging();

    LoggerPanel panel(32);
    auto logger = utils::get_logger("editor.logger_panel.test");

    const auto before_count = panel.buffered_count();
    logger->info("logger-panel-realtime-token");

    EXPECT_GT(panel.buffered_count(), before_count);
    EXPECT_TRUE(panel.contains_message("logger-panel-realtime-token"));

    utils::shutdown_logging();
}

TEST(LoggerPanelTest, DestructionUnsubscribesWithoutDanglingCallback) {
    init_test_logging();
    auto logger = utils::get_logger("editor.logger_panel.test");

    {
        auto panel = std::make_unique<LoggerPanel>(16);
        logger->info("logger-panel-before-destroy");
        EXPECT_TRUE(panel->contains_message("logger-panel-before-destroy"));
    }

    logger->info("logger-panel-after-destroy");
    SUCCEED();

    utils::shutdown_logging();
}

} // namespace rtr::editor::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
