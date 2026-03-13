#pragma once

#include <tuple>

#include "rtr/framework/integration/pbpt/serde/load/mappers.hpp"
#include "rtr/framework/integration/pbpt/serde/write/mappers.hpp"

namespace rtr::framework::integration {

using ShapeLoaderList      = std::tuple<ObjLambertianShapeImportMapper>;
using CameraLoaderList     = std::tuple<ThinLensPerspectiveImportMapper>;
using IntegratorLoaderList = std::tuple<SimplePathIntegratorImportMapper>;

using ShapeWriterList = std::tuple<StaticMeshComponentPbptMeshExportMapper>;

// compile-time verification
static_assert(ImportShapeMapperConcept<ObjLambertianShapeImportMapper>,
              "ObjLambertianShapeImportMapper violates concept");
static_assert(ImportCameraMapperConcept<ThinLensPerspectiveImportMapper>,
              "ThinLensPerspectiveImportMapper violates concept");
static_assert(ImportIntegratorMapperConcept<SimplePathIntegratorImportMapper>, "SimplePathIntegratorImportMapper violates concept");
static_assert(ExportShapeMapperConcept<StaticMeshComponentPbptMeshExportMapper>,
              "StaticMeshComponentPbptMeshExportMapper violates concept");

}  // namespace rtr::framework::integration
