#pragma once

#include <array>
#include <string_view>

#include "imgui.h"
#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/editor/render/shadertoy_editor_pipeline.hpp"

namespace rtr::editor {

class ShaderToySettingsPanel final : public IEditorPanel {
private:
    render::ShaderToyEditorPipeline* m_pipeline{nullptr};
    bool                             m_visible{true};

public:
    explicit ShaderToySettingsPanel(render::ShaderToyEditorPipeline* pipeline) : m_pipeline(pipeline) {}

    std::string_view id() const override { return "shadertoy_settings"; }
    int              order() const override { return 100; }
    bool             visible() const override { return m_visible; }
    void             set_visible(bool visible) override { m_visible = visible; }

    void on_imgui(EditorContext& /*ctx*/) override {
        if (!m_visible || !m_pipeline) {
            return;
        }

        if (ImGui::Begin("ShaderToy Settings", &m_visible)) {
            auto& params = m_pipeline->params();
            ImGui::SliderFloat("Param 0", &params[0], 0.0f, 10.0f);
            ImGui::SliderFloat("Param 1", &params[1], 0.0f, 10.0f);
            ImGui::SliderFloat("Param 2", &params[2], 0.0f, 10.0f);
            ImGui::SliderFloat("Param 3", &params[3], 0.0f, 10.0f);

            ImGui::Separator();
            if (ImGui::Button("Reset")) {
                params = {1.0f, 0.0f, 0.0f, 0.0f};
            }
        }
        ImGui::End();
    }
};

}  // namespace rtr::editor
