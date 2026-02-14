#pragma once

#include <cmath>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace rtr::framework::component {

struct PbptSpectrumPoint {
    float lambda_nm{0.0f};
    float value{0.0f};
};

using PbptSpectrum = std::vector<PbptSpectrumPoint>;

inline PbptSpectrum make_constant_pbpt_spectrum(float value) {
    return {
        PbptSpectrumPoint{400.0f, value},
        PbptSpectrumPoint{500.0f, value},
        PbptSpectrumPoint{600.0f, value},
        PbptSpectrumPoint{700.0f, value},
    };
}

inline void validate_pbpt_spectrum(
    const PbptSpectrum& spectrum,
    std::string_view field_name = "spectrum"
) {
    if (spectrum.empty()) {
        throw std::invalid_argument(std::string(field_name) + " must not be empty.");
    }

    float last_lambda = -1.0f;
    for (std::size_t i = 0; i < spectrum.size(); ++i) {
        const auto& point = spectrum[i];
        if (!std::isfinite(point.lambda_nm) || point.lambda_nm <= 0.0f) {
            throw std::invalid_argument(
                std::string(field_name) + " has invalid lambda at index " + std::to_string(i) + "."
            );
        }
        if (!std::isfinite(point.value) || point.value < 0.0f) {
            throw std::invalid_argument(
                std::string(field_name) + " has invalid value at index " + std::to_string(i) + "."
            );
        }
        if (i > 0 && point.lambda_nm <= last_lambda) {
            throw std::invalid_argument(
                std::string(field_name) + " lambda must be strictly increasing."
            );
        }
        last_lambda = point.lambda_nm;
    }
}

inline std::string serialize_pbpt_spectrum(const PbptSpectrum& spectrum) {
    validate_pbpt_spectrum(spectrum);

    std::ostringstream oss;
    oss.setf(std::ios::fixed);
    oss << std::setprecision(6);

    for (std::size_t i = 0; i < spectrum.size(); ++i) {
        if (i > 0) {
            oss << ", ";
        }
        oss << spectrum[i].lambda_nm << ":" << spectrum[i].value;
    }
    return oss.str();
}

} // namespace rtr::framework::component
