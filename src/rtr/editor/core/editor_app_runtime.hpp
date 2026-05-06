#pragma once

#include "rtr/app/app_runtime.hpp"
#include "rtr/editor/render/editor_output_backend.hpp"

namespace rtr::editor {

using AppRuntime = app::AppRuntimeT<render::EditorOutputBackend>;
using EditorAppRuntime = AppRuntime;

}  // namespace rtr::editor
