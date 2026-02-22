#include <gtest/gtest.h>
#include <pbpt/math/math.h>

#include "rtr/framework/component/light/point_light.hpp"
#include "rtr/framework/core/scene.hpp"

using namespace rtr::framework;
using namespace rtr::framework::component::light;

class FrameworkPointLightTest : public ::testing::Test {
protected:
    core::Scene       scene{};
    core::GameObject* go{nullptr};

    void SetUp() override { go = &scene.create_game_object("test_go"); }
};

TEST_F(FrameworkPointLightTest, DefaultValues) {
    auto& light = go->add_component<PointLight>();
    EXPECT_FLOAT_EQ(light.color.x(), 1.0f);
    EXPECT_FLOAT_EQ(light.color.y(), 1.0f);
    EXPECT_FLOAT_EQ(light.color.z(), 1.0f);
    EXPECT_FLOAT_EQ(light.intensity, 10.0f);
    EXPECT_FLOAT_EQ(light.range, 10.0f);
    EXPECT_FLOAT_EQ(light.specular_strength, 1.0f);
    EXPECT_FLOAT_EQ(light.shininess, 32.0f);
    EXPECT_TRUE(light.enabled());
}

TEST_F(FrameworkPointLightTest, SettersAndGetters) {
    auto& light = go->add_component<PointLight>();

    light.set_color({0.5f, 0.2f, 0.1f});
    EXPECT_FLOAT_EQ(light.color.x(), 0.5f);
    EXPECT_FLOAT_EQ(light.color.y(), 0.2f);
    EXPECT_FLOAT_EQ(light.color.z(), 0.1f);

    light.set_intensity(5.0f);
    EXPECT_FLOAT_EQ(light.intensity, 5.0f);

    light.set_range(20.0f);
    EXPECT_FLOAT_EQ(light.range, 20.0f);

    light.set_specular_strength(2.0f);
    EXPECT_FLOAT_EQ(light.specular_strength, 2.0f);

    light.set_shininess(64.0f);
    EXPECT_FLOAT_EQ(light.shininess, 64.0f);
}

TEST_F(FrameworkPointLightTest, InvalidParametersThrow) {
    auto& light = go->add_component<PointLight>();

    EXPECT_THROW(light.set_intensity(-1.0f), std::invalid_argument);
    EXPECT_THROW(light.set_range(0.0f), std::invalid_argument);
    EXPECT_THROW(light.set_range(-5.0f), std::invalid_argument);
    EXPECT_THROW(light.set_specular_strength(-0.5f), std::invalid_argument);
    EXPECT_THROW(light.set_shininess(0.5f), std::invalid_argument);
    EXPECT_THROW(light.set_shininess(0.0f), std::invalid_argument);
}

TEST_F(FrameworkPointLightTest, UniqueComponentConstraint) {
    go->add_component<PointLight>();
    EXPECT_THROW(go->add_component<PointLight>(), std::runtime_error);
}
