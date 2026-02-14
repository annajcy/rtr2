#pragma once

#include <algorithm>
#include <type_traits>
#include <utility>
#include <vector>

#include "pbpt/radiometry/color.hpp"
#include "pbpt/radiometry/color_spectrum_lut.hpp"
#include "pbpt/radiometry/constant/illuminant_spectrum.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/rgb_sigmoid.hpp"
#include "pbpt/radiometry/plugin/spectrum_distribution/rgb_spectra.hpp"
#include "rtr/framework/component/pbpt/pbpt_mesh.hpp"
#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"

namespace rtr::framework::integration {

namespace detail {

inline pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double> to_piecewise_spectrum(
    const component::PbptSpectrum& spectrum
) {
    component::validate_pbpt_spectrum(spectrum, "PbptReflectanceConvert.spectrum");
    std::vector<std::pair<double, double>> points{};
    points.reserve(spectrum.size());
    for (const auto& point : spectrum) {
        points.emplace_back(
            static_cast<double>(point.lambda_nm),
            static_cast<double>(point.value)
        );
    }
    return pbpt::radiometry::PiecewiseLinearSpectrumDistribution<double>(points);
}

inline float clamp_unit(float value) {
    return std::clamp(value, 0.0f, 1.0f);
}

} // namespace detail

inline component::PbptRgb pbpt_spectrum_to_rgb(const component::PbptSpectrum& spectrum) {
    const auto piecewise = detail::to_piecewise_spectrum(spectrum);
    const auto xyz = pbpt::radiometry::XYZ<double>::from_reflectance(
        piecewise,
        pbpt::radiometry::constant::CIE_D65_ilum<double>
    );
    const auto linear_rgb = pbpt::radiometry::constant::sRGB<double>.to_rgb(xyz);
    return component::PbptRgb{
        .r = detail::clamp_unit(static_cast<float>(linear_rgb.r())),
        .g = detail::clamp_unit(static_cast<float>(linear_rgb.g())),
        .b = detail::clamp_unit(static_cast<float>(linear_rgb.b()))
    };
}

inline component::PbptSpectrum pbpt_rgb_to_spectrum(const component::PbptRgb& rgb) {
    component::validate_pbpt_rgb(rgb, "PbptReflectanceConvert.rgb");
    const pbpt::radiometry::RGB<double> srgb_rgb{
        static_cast<double>(rgb.r),
        static_cast<double>(rgb.g),
        static_cast<double>(rgb.b)
    };
    const auto rsp = pbpt::radiometry::lookup_srgb_to_rsp(srgb_rgb);
    const pbpt::radiometry::RGBAlbedoSpectrumDistribution<
        double,
        pbpt::radiometry::RGBSigmoidPolynomialNormalized
    > albedo_spectrum(rsp);

    component::PbptSpectrum piecewise{};
    piecewise.reserve(830 - 360 + 1);
    for (int lambda_nm = 360; lambda_nm <= 830; ++lambda_nm) {
        piecewise.emplace_back(component::PbptSpectrumPoint{
            .lambda_nm = static_cast<float>(lambda_nm),
            .value = static_cast<float>(albedo_spectrum.at(static_cast<double>(lambda_nm)))
        });
    }
    component::validate_pbpt_spectrum(piecewise, "PbptReflectanceConvert.spectrum");
    return piecewise;
}

inline component::PbptRgb pbpt_reflectance_to_rgb(const component::PbptReflectance& reflectance) {
    return std::visit(
        [](const auto& source) -> component::PbptRgb {
            using SourceT = std::decay_t<decltype(source)>;
            if constexpr (std::is_same_v<SourceT, component::PbptSpectrum>) {
                return pbpt_spectrum_to_rgb(source);
            } else {
                return source;
            }
        },
        reflectance
    );
}

inline component::PbptSpectrum pbpt_reflectance_to_spectrum(
    const component::PbptReflectance& reflectance
) {
    return std::visit(
        [](const auto& source) -> component::PbptSpectrum {
            using SourceT = std::decay_t<decltype(source)>;
            if constexpr (std::is_same_v<SourceT, component::PbptRgb>) {
                return pbpt_rgb_to_spectrum(source);
            } else {
                component::validate_pbpt_spectrum(source, "PbptReflectanceConvert.spectrum");
                return source;
            }
        },
        reflectance
    );
}

} // namespace rtr::framework::integration
