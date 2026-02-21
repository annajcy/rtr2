#pragma once

#include <concepts>
#include <string_view>

#include "pbpt/serde/scene_loader.hpp"
#include "rtr/framework/core/game_object.hpp"
#include "rtr/framework/integration/pbpt/bridge/context.hpp"

namespace rtr::framework::integration {

struct PbptImportPackage;

template <typename T>
concept ImportShapeMapperConcept = requires(const pbpt::scene::ShapeInstanceRecord<float>& record,
                                            const ImportGlobalContext& ctx, PbptImportPackage& pkg) {
    { T::kName } -> std::convertible_to<std::string_view>;
    { T::matches(record, ctx, pkg) } -> std::same_as<bool>;
    { T::map(record, ctx, pkg) } -> std::same_as<void>;
};

template <typename T>
concept ExportShapeMapperConcept =
    requires(const core::GameObject& go, const ExportGlobalContext& ctx, pbpt::serde::PbptXmlResult<float>& result) {
        { T::kName } -> std::convertible_to<std::string_view>;
        { T::matches(go, ctx, result) } -> std::same_as<bool>;
        { T::map(go, ctx, result) } -> std::same_as<void>;
    };

template <typename T>
concept ImportCameraMapperConcept =
    requires(const pbpt::camera::AnyCamera<float>& camera, const ImportGlobalContext& ctx, PbptImportPackage& pkg) {
        { T::kName } -> std::convertible_to<std::string_view>;
        { T::matches(camera, ctx, pkg) } -> std::same_as<bool>;
        { T::map(camera, ctx, pkg) } -> std::same_as<void>;
    };

template <typename T>
concept ImportIntegratorMapperConcept = requires(const pbpt::integrator::AnyIntegrator<float>& integrator,
                                                 const ImportGlobalContext& ctx, PbptImportPackage& pkg) {
    { T::kName } -> std::convertible_to<std::string_view>;
    { T::matches(integrator, ctx, pkg) } -> std::same_as<bool>;
    { T::map(integrator, ctx, pkg) } -> std::same_as<void>;
};

}  // namespace rtr::framework::integration
