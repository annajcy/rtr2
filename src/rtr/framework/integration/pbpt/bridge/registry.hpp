#pragma once

#include <tuple>

#include "rtr/framework/integration/pbpt/bridge/import_mappers.hpp"
#include "rtr/framework/integration/pbpt/bridge/export_mappers.hpp"

namespace rtr::framework::integration {

using ImportShapeMapperList      = std::tuple<ObjLambertianShapeImportMapper>;
using ImportCameraMapperList     = std::tuple<ThinLensPerspectiveImportMapper>;
using ImportIntegratorMapperList = std::tuple<PathIntegratorImportMapper>;

using ExportShapeMapperList = std::tuple<MeshRendererPbptMeshExportMapper>;

// compile-time verification
static_assert(ImportShapeMapperConcept<ObjLambertianShapeImportMapper>,
              "ObjLambertianShapeImportMapper violates concept");
static_assert(ImportCameraMapperConcept<ThinLensPerspectiveImportMapper>,
              "ThinLensPerspectiveImportMapper violates concept");
static_assert(ImportIntegratorMapperConcept<PathIntegratorImportMapper>, "PathIntegratorImportMapper violates concept");
static_assert(ExportShapeMapperConcept<MeshRendererPbptMeshExportMapper>,
              "MeshRendererPbptMeshExportMapper violates concept");

}  // namespace rtr::framework::integration
