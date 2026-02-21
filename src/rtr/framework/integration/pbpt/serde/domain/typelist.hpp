#pragma once

#include <tuple>

#include "rtr/framework/integration/pbpt/serde/load/mappers.hpp"
#include "rtr/framework/integration/pbpt/serde/write/mappers.hpp"

namespace rtr::framework::integration {

using ShapeLoaderList      = std::tuple<ObjLambertianShapeImportMapper>;
using CameraLoaderList     = std::tuple<ThinLensPerspectiveImportMapper>;
using IntegratorLoaderList = std::tuple<PathIntegratorImportMapper>;

using ShapeWriterList = std::tuple<MeshRendererPbptMeshExportMapper>;

// compile-time verification
static_assert(ImportShapeMapperConcept<ObjLambertianShapeImportMapper>,
              "ObjLambertianShapeImportMapper violates concept");
static_assert(ImportCameraMapperConcept<ThinLensPerspectiveImportMapper>,
              "ThinLensPerspectiveImportMapper violates concept");
static_assert(ImportIntegratorMapperConcept<PathIntegratorImportMapper>, "PathIntegratorImportMapper violates concept");
static_assert(ExportShapeMapperConcept<MeshRendererPbptMeshExportMapper>,
              "MeshRendererPbptMeshExportMapper violates concept");

}  // namespace rtr::framework::integration
