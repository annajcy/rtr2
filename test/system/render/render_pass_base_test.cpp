#include <stdexcept>

#include "gtest/gtest.h"

#include "rtr/system/render/render_pass.hpp"

namespace rtr::system::render::test {

namespace {

struct ProbeResources {
    vk::Extent2D extent{};
    int value{0};
};

class ProbePass final : public RenderPass<ProbeResources> {
public:
    bool validate_called{false};
    bool execute_called{false};
    int observed_value{0};

protected:
    void validate(const ProbeResources& resources) const override {
        const_cast<ProbePass*>(this)->validate_called = true;
        require_valid_extent(resources.extent, "ProbeResources extent is invalid.");
    }

    void do_execute(FrameContext&, const ProbeResources& resources) override {
        execute_called = true;
        observed_value = resources.value;
    }
};

} // namespace

TEST(RenderPassBaseTest, ExecuteRunsValidateThenDoExecute) {
    ProbePass pass;
    auto& ctx = *reinterpret_cast<FrameContext*>(0x1);

    pass.execute(ctx, ProbeResources{.extent = vk::Extent2D{128, 64}, .value = 42});

    EXPECT_TRUE(pass.validate_called);
    EXPECT_TRUE(pass.execute_called);
    EXPECT_EQ(pass.observed_value, 42);
}

TEST(RenderPassBaseTest, ValidateFailureSkipsDoExecute) {
    ProbePass pass;
    auto& ctx = *reinterpret_cast<FrameContext*>(0x1);

    EXPECT_THROW(
        pass.execute(ctx, ProbeResources{.extent = vk::Extent2D{0, 64}, .value = 7}),
        std::runtime_error
    );

    EXPECT_TRUE(pass.validate_called);
    EXPECT_FALSE(pass.execute_called);
}

} // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
