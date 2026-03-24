#pragma once

#include <array>
#include <cstdio>
#include <string>
#include <string_view>

#include "imgui.h"
#include "rtr/editor/core/editor_panel.hpp"
#include "rtr/system/render/pipeline/shadertoy/editable_shadertoy_pipeline.hpp"

namespace rtr::editor {

class EditableShaderToySettingsPanel final : public IEditorPanel {
private:
    static constexpr std::size_t kPathBufferSize = 1024;

    system::render::EditableShaderToyPipeline* m_pipeline{nullptr};
    bool m_visible{true};
    std::array<char, kPathBufferSize> m_path_buffer{};

    static const char* phase_label(system::render::EditableShaderToyReloadPhase phase) {
        switch (phase) {
        case system::render::EditableShaderToyReloadPhase::Uninitialized:
            return "Uninitialized";
        case system::render::EditableShaderToyReloadPhase::Compiling:
            return "Compiling";
        case system::render::EditableShaderToyReloadPhase::Ready:
            return "Ready";
        case system::render::EditableShaderToyReloadPhase::MissingFile:
            return "Missing File";
        case system::render::EditableShaderToyReloadPhase::Error:
            return "Error";
        }
        return "Unknown";
    }

    void sync_path_buffer() {
        if (m_pipeline == nullptr) {
            return;
        }
        std::snprintf(m_path_buffer.data(), m_path_buffer.size(), "%s", m_pipeline->shader_source_path().c_str());
    }

public:
    explicit EditableShaderToySettingsPanel(system::render::EditableShaderToyPipeline* pipeline)
        : m_pipeline(pipeline) {
        sync_path_buffer();
    }

    std::string_view id() const override { return "editable_shadertoy_settings"; }
    int order() const override { return 100; }
    bool visible() const override { return m_visible; }
    void set_visible(bool visible) override { m_visible = visible; }

    void on_imgui(EditorContext& /*ctx*/) override {
        if (!m_visible || m_pipeline == nullptr) {
            return;
        }

        if (!ImGui::Begin("Editable ShaderToy Settings", &m_visible)) {
            ImGui::End();
            return;
        }

        if (std::string(m_path_buffer.data()) != m_pipeline->shader_source_path()) {
            sync_path_buffer();
        }

        if (ImGui::InputText("Shader Source", m_path_buffer.data(), m_path_buffer.size())) {
            m_pipeline->set_shader_source_path(m_path_buffer.data());
        }

        bool auto_reload = m_pipeline->auto_reload_enabled();
        if (ImGui::Checkbox("Auto Reload", &auto_reload)) {
            m_pipeline->set_auto_reload_enabled(auto_reload);
        }
        ImGui::SameLine();
        if (ImGui::Button("Reload")) {
            m_pipeline->request_shader_reload();
        }

        const auto& reload_state = m_pipeline->reload_state();
        ImGui::Separator();
        ImGui::Text("Status: %s", phase_label(reload_state.phase));
        ImGui::Text("Program Valid: %s", reload_state.has_valid_program ? "Yes" : "No");
        ImGui::TextWrapped("Resolved Path: %s", reload_state.resolved_path.empty()
                                                    ? "(unresolved)"
                                                    : reload_state.resolved_path.string().c_str());

        if (!reload_state.last_error_message.empty()) {
            ImGui::Spacing();
            ImGui::TextColored(ImVec4(1.0f, 0.45f, 0.45f, 1.0f), "Latest Error");
            ImGui::BeginChild("##editable_shadertoy_error", ImVec2(0.0f, 120.0f), true);
            ImGui::TextUnformatted(reload_state.last_error_message.c_str());
            ImGui::EndChild();
        }

        auto& params = m_pipeline->params();
        ImGui::Separator();
        ImGui::SliderFloat("Param 0", &params[0], 0.0f, 10.0f);
        ImGui::SliderFloat("Param 1", &params[1], 0.0f, 10.0f);
        ImGui::SliderFloat("Param 2", &params[2], 0.0f, 10.0f);
        ImGui::SliderFloat("Param 3", &params[3], 0.0f, 10.0f);

        if (ImGui::Button("Reset")) {
            params = {1.0f, 0.0f, 0.0f, 0.0f};
        }

        ImGui::End();
    }
};

}  // namespace rtr::editor
