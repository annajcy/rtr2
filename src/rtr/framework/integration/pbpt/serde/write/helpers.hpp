#pragma once

#include <pbpt/math/math.h>
#include <pbpt/geometry/transform.hpp>
#include <pbpt/material/plugin/material/lambertian_material.hpp>
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"

#include <iomanip>
#include <cstdint>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/utils/obj_io.hpp"

namespace rtr::framework::integration::compat_export_detail {

inline std::string rgb_value_string(const component::PbptRgb& rgb) {
    component::validate_pbpt_rgb(rgb, "PbptShapeRecord.reflectance_rgb");
    std::ostringstream oss;
    oss << std::setprecision(6) << rgb.r << " " << rgb.g << " " << rgb.b;
    return oss.str();
}

inline std::string reflectance_key(const component::PbptRgb& reflectance) {
    return "rgb:" + rgb_value_string(reflectance);
}

inline ::pbpt::math::mat4 to_mat4(const ::pbpt::geometry::Transform<float>& transform) {
    ::pbpt::math::mat4 matrix{1.0f};
    const auto&      src = transform.matrix();
    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            matrix[row][col] = src.at(row, col);
        }
    }
    return matrix;
}

inline ::pbpt::geometry::Transform<float> to_transform(const ::pbpt::math::mat4& matrix) {
    return ::pbpt::geometry::Transform<float>(matrix);
}

inline component::PbptSpectrum to_component_spectrum(
    const ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>& spectrum) {
    component::PbptSpectrum out{};
    out.reserve(spectrum.points().size());
    for (const auto& [lambda, value] : spectrum.points()) {
        out.emplace_back(component::PbptSpectrumPoint{.lambda_nm = lambda, .value = value});
    }
    component::validate_pbpt_spectrum(out, "piecewise_spectrum");
    return out;
}

inline ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float> to_piecewise_spectrum(
    const component::PbptSpectrum& spectrum) {
    component::validate_pbpt_spectrum(spectrum, "pbpt_light.radiance_spectrum");
    std::vector<std::pair<float, float>> points{};
    points.reserve(spectrum.size());
    for (const auto& point : spectrum) {
        points.emplace_back(point.lambda_nm, point.value);
    }
    return ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>(points);
}

inline ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float> rgb_to_piecewise(const component::PbptRgb& rgb) {
    const auto spectrum = ::rtr::framework::integration::pbpt_rgb_to_spectrum(rgb);
    return to_piecewise_spectrum(spectrum);
}

template <typename ExistsFn>
std::string make_unique_name(std::string base, ExistsFn&& exists) {
    if (!exists(base)) {
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!exists(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("Failed to generate unique resource name.");
}

inline std::string make_unique_shape_id(std::string base, std::unordered_set<std::string>& used_shape_ids) {
    if (base.empty()) {
        base = "shape";
    }
    if (!used_shape_ids.contains(base)) {
        used_shape_ids.insert(base);
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!used_shape_ids.contains(candidate)) {
            used_shape_ids.insert(candidate);
            return candidate;
        }
    }
    throw std::runtime_error("Failed to generate unique shape id.");
}

inline ::pbpt::shape::TriangleMesh<float> to_pbpt_triangle_mesh(
    const utils::ObjMeshData& mesh, const ::pbpt::camera::RenderTransform<float>& render_transform,
    const ::pbpt::geometry::Transform<float>& object_to_world) {
    if (mesh.vertices.empty() || mesh.indices.empty()) {
        throw std::runtime_error("Cannot convert empty RTR mesh to PBPT mesh.");
    }

    std::vector<int> indices{};
    indices.reserve(mesh.indices.size());
    for (const auto idx : mesh.indices) {
        if (idx > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
            throw std::runtime_error("RTR mesh index exceeds int range for PBPT conversion.");
        }
        indices.emplace_back(static_cast<int>(idx));
    }

    std::vector<::pbpt::math::Point<float, 3>>  positions{};
    std::vector<::pbpt::math::Normal<float, 3>> normals{};
    std::vector<::pbpt::math::Point<float, 2>>  uvs{};
    positions.reserve(mesh.vertices.size());
    normals.reserve(mesh.vertices.size());
    uvs.reserve(mesh.vertices.size());
    for (const auto& vertex : mesh.vertices) {
        positions.emplace_back(vertex.position.x(), vertex.position.y(), vertex.position.z());
        normals.emplace_back(vertex.normal.x(), vertex.normal.y(), vertex.normal.z());
        uvs.emplace_back(vertex.uv.x(), vertex.uv.y());
    }

    return ::pbpt::shape::TriangleMesh<float>(render_transform, indices, positions, normals, uvs, false, object_to_world);
}

}  // namespace rtr::framework::integration::compat_export_detail
