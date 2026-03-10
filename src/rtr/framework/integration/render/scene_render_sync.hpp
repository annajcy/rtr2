#pragma once

#include "rtr/editor/render/editable_shadertoy_editor_pipeline.hpp"
#include "rtr/editor/render/forward_editor_pipeline.hpp"
#include "rtr/framework/core/world.hpp"
#include "rtr/resource/resource_manager.hpp"
#include "rtr/system/input/input_system.hpp"
#include "rtr/system/render/pipeline/forward/forward_pipeline.hpp"

#include <stdexcept>

namespace rtr::framework::integration::render {

struct RenderFrameContext {
    framework::core::World&     world;
    resource::ResourceManager&  resources;
    system::input::InputSystem& input;
    std::uint64_t               frame_serial{0};
    double                      delta_seconds{0.0};
};

// Default no-op: ShaderToyPipeline, ShaderToyEditorPipeline, and any future
// pipeline that does not need scene data.
template <typename TPipeline>
void sync_scene_to_render(TPipeline& /*pipeline*/, const RenderFrameContext& /*ctx*/) {}

// ForwardPipeline: extract scene view from active scene.
inline void sync_scene_to_render(system::render::ForwardPipeline& pipeline,
                                  const RenderFrameContext& ctx) {
    auto* scene = ctx.world.active_scene();
    if (!scene)
        throw std::runtime_error("sync_scene_to_render: no active scene.");
    pipeline.prepare_scene(*scene, ctx.resources);
}

// ForwardEditorPipeline: same extraction.
inline void sync_scene_to_render(editor::render::ForwardEditorPipeline& pipeline,
                                  const RenderFrameContext& ctx) {
    auto* scene = ctx.world.active_scene();
    if (!scene)
        throw std::runtime_error("sync_scene_to_render: no active scene.");
    pipeline.prepare_scene(*scene, ctx.resources);
}

// EditableShaderToyEditorPipeline: trigger shader hot-reload check (no scene data needed).
inline void sync_scene_to_render(editor::render::EditableShaderToyEditorPipeline& pipeline,
                                  const RenderFrameContext& /*ctx*/) {
    pipeline.check_and_reload_shader();
}

} // namespace rtr::framework::integration::render
