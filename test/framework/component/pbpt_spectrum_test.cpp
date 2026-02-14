#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"

namespace rtr::framework::component::test {

TEST(PbptSpectrumTest, MakeConstantSpectrumUsesDefaultWavelengthGrid) {
    const auto spectrum = make_constant_pbpt_spectrum(0.7f);
    ASSERT_EQ(spectrum.size(), 4u);

    EXPECT_FLOAT_EQ(spectrum[0].lambda_nm, 400.0f);
    EXPECT_FLOAT_EQ(spectrum[1].lambda_nm, 500.0f);
    EXPECT_FLOAT_EQ(spectrum[2].lambda_nm, 600.0f);
    EXPECT_FLOAT_EQ(spectrum[3].lambda_nm, 700.0f);

    for (const auto& point : spectrum) {
        EXPECT_FLOAT_EQ(point.value, 0.7f);
    }
}

TEST(PbptSpectrumTest, ValidateRejectsInvalidInputs) {
    EXPECT_THROW(
        (void)validate_pbpt_spectrum({}, "s"),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)validate_pbpt_spectrum({
            {400.0f, 1.0f},
            {300.0f, 1.0f},
        }, "s"),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)validate_pbpt_spectrum({
            {400.0f, -0.1f},
            {500.0f, 0.2f},
        }, "s"),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)validate_pbpt_spectrum({
            {400.0f, std::numeric_limits<float>::infinity()},
            {500.0f, 0.2f},
        }, "s"),
        std::invalid_argument
    );

    EXPECT_THROW(
        (void)validate_pbpt_spectrum({
            {400.0f, 0.2f},
            {std::numeric_limits<float>::quiet_NaN(), 0.2f},
        }, "s"),
        std::invalid_argument
    );
}

TEST(PbptSpectrumTest, SerializeUsesStableFixedPrecisionFormat) {
    const PbptSpectrum spectrum{
        {400.0f, 0.7f},
        {500.0f, 1.0f},
        {600.0f, 0.3333333f},
    };

    const auto serialized = serialize_pbpt_spectrum(spectrum);
    EXPECT_EQ(
        serialized,
        "400.000000:0.700000, 500.000000:1.000000, 600.000000:0.333333"
    );
}

} // namespace rtr::framework::component::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
