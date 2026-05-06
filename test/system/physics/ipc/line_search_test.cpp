#include <gtest/gtest.h>

#include "rtr/system/physics/ipc/solver/line_search.hpp"

namespace rtr::system::physics::ipc {
namespace {

TEST(LineSearchTest, AcceptsFullStepWhenEnergyDrops) {
    const auto result = backtracking_line_search(
        [](double alpha) { return (1.0 - alpha) * (1.0 - alpha); },
        1.0,
        -2.0
    );

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(result.alpha, 1.0);
    EXPECT_DOUBLE_EQ(result.energy, 0.0);
}

TEST(LineSearchTest, ShrinksStepUntilArmijoConditionHolds) {
    const auto result = backtracking_line_search(
        [](double alpha) {
            if (alpha > 0.25) {
                return 1.0;
            }
            return -0.01 * alpha;
        },
        0.0,
        -1.0,
        1.0,
        0.5
    );

    EXPECT_TRUE(result.success);
    EXPECT_DOUBLE_EQ(result.alpha, 0.25);
    EXPECT_DOUBLE_EQ(result.energy, -0.0025);
}

TEST(LineSearchTest, RejectsNonDescentDirectionsImmediately) {
    const auto result = backtracking_line_search(
        [](double alpha) { return alpha * alpha; },
        1.0,
        0.0
    );

    EXPECT_FALSE(result.success);
    EXPECT_DOUBLE_EQ(result.alpha, 0.0);
    EXPECT_DOUBLE_EQ(result.energy, 1.0);
}

}  // namespace
}  // namespace rtr::system::physics::ipc
