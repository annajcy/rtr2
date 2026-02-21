#pragma once

#include <pbpt/math/math.h>
#include <pbpt/geometry/transform.hpp>
#include <pbpt/material/plugin/material/lambertian_material.hpp>
#include "pbpt/radiometry/plugin/spectrum_distribution/piecewise_linear.hpp"
#include "pbpt/radiometry/constant/standard_color_spaces.hpp"
#include "pbpt/shape/plugin/shape/triangle.hpp"

#include <string>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <variant>
#include <vector>

#include "rtr/framework/component/pbpt/pbpt_spectrum.hpp"
#include "rtr/framework/integration/pbpt/pbpt_reflectance_convert.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/utils/obj_io.hpp"
#include "rtr/framework/integration/pbpt/serde/load/types.hpp"

namespace rtr::framework::integration::compat_import_detail {

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

inline component::PbptRgb lambertian_to_rgb(const ::pbpt::material::LambertianMaterial<float>& material) {
    const auto& source = material.reflectance_source();
    return std::visit(
        [](const auto& value) -> component::PbptRgb {
            using ValueT = std::decay_t<decltype(value)>;
            if constexpr (std::is_same_v<ValueT, ::pbpt::radiometry::PiecewiseLinearSpectrumDistribution<float>>) {
                return ::rtr::framework::integration::pbpt_spectrum_to_rgb(to_component_spectrum(value));
            } else {
                throw std::runtime_error("Lambertian texture reflectance is not expressible by RTR MeshRenderer.");
            }
        },
        source);
}

inline utils::ObjMeshData to_rtr_mesh_data(const ::pbpt::shape::TriangleMesh<float>& mesh) {
    utils::ObjMeshData out{};
    const auto         render_to_object = mesh.render_to_object_transform();

    const auto& positions = mesh.positions();
    const auto& uvs       = mesh.uvs();
    const auto& normals   = mesh.normals();
    out.vertices.resize(positions.size());

    // Convert positions and UVs back to object space.
    for (std::size_t i = 0; i < positions.size(); ++i) {
        const auto object_p      = render_to_object.transform_point(positions[i]);
        out.vertices[i].position = ::pbpt::math::vec3(object_p.x(), object_p.y(), object_p.z());

        if (mesh.has_uvs() && i < uvs.size()) {
            out.vertices[i].uv = ::pbpt::math::vec2(uvs[i].x(), uvs[i].y());
        } else {
            out.vertices[i].uv = ::pbpt::math::vec2(0.0f, 0.0f);
        }
    }

    // Build index list first (needed for normal generation).
    out.indices.reserve(mesh.indices().size());
    for (const int idx : mesh.indices()) {
        if (idx < 0) {
            throw std::runtime_error("PBPT mesh contains negative index.");
        }
        out.indices.emplace_back(static_cast<std::uint32_t>(idx));
    }

    if (mesh.has_normals()) {
        // Transform explicit normals back to object space, applying flip
        // to match the runtime orientation used by Triangle::intersect.
        const bool flip = mesh.should_flip_normal();
        for (std::size_t i = 0; i < positions.size(); ++i) {
            auto object_n = render_to_object.transform_normal(normals[i]).normalized();
            if (flip) {
                object_n = -object_n;
            }
            out.vertices[i].normal = ::pbpt::math::vec3(object_n.x(), object_n.y(), object_n.z());
        }
    } else {
        // No normals in the PBPT mesh — compute them from geometry winding,
        // the same way obj_io.hpp does it.
        std::vector<::pbpt::math::vec3> accum(out.vertices.size(), ::pbpt::math::vec3(0.0f));
        for (std::size_t i = 0; i + 2u < out.indices.size(); i += 3u) {
            const auto  i0     = out.indices[i + 0u];
            const auto  i1     = out.indices[i + 1u];
            const auto  i2     = out.indices[i + 2u];
            const auto& p0     = out.vertices[i0].position;
            const auto& p1     = out.vertices[i1].position;
            const auto& p2     = out.vertices[i2].position;
            const auto  face_n = ::pbpt::math::normalize(::pbpt::math::cross(p1 - p0, p2 - p0));
            accum[i0] += face_n;
            accum[i1] += face_n;
            accum[i2] += face_n;
        }
        for (std::size_t v = 0; v < out.vertices.size(); ++v) {
            const float len        = ::pbpt::math::length(accum[v]);
            out.vertices[v].normal = len > 0.0f ? ::pbpt::math::normalize(accum[v]) : ::pbpt::math::vec3(0.0f, 1.0f, 0.0f);
        }
    }

    if (out.vertices.empty() || out.indices.empty()) {
        throw std::runtime_error("PBPT mesh conversion produced empty ObjMeshData.");
    }
    return out;
}

inline std::string make_unique_name(const std::string&                                         base,
                                    const std::unordered_map<std::string, core::GameObjectId>& existing) {
    if (!existing.contains(base)) {
        return base;
    }
    for (int suffix = 1; suffix < std::numeric_limits<int>::max(); ++suffix) {
        const std::string candidate = base + "_" + std::to_string(suffix);
        if (!existing.contains(candidate)) {
            return candidate;
        }
    }
    throw std::runtime_error("Failed to create unique imported GameObject name.");
}

inline void register_imported_game_object(LoadSummary& result, const std::string& name, core::GameObjectId id) {
    auto [_, inserted] = result.imported_game_object_id_by_name.emplace(name, id);
    if (!inserted) {
        throw std::runtime_error("Duplicate imported game object name: " + name);
    }
}

}  // namespace rtr::framework::integration::compat_import_detail
