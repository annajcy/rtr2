#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include "rtr/system/render/utils/shader_compiler.hpp"
#include "rtr/utils/log.hpp"

namespace rtr::system::render {

enum class EditableShaderToyReloadPhase {
    Uninitialized,
    Compiling,
    Ready,
    MissingFile,
    Error
};

struct EditableShaderToyReloadState {
    EditableShaderToyReloadPhase phase{EditableShaderToyReloadPhase::Uninitialized};
    std::filesystem::path resolved_path{};
    std::optional<std::filesystem::file_time_type> last_write_time{};
    std::string last_error_message{};
    bool has_valid_program{false};
};

struct EditableShaderToyReloadCheckResult {
    bool should_compile{false};
    std::filesystem::path resolved_path{};
    std::optional<std::filesystem::file_time_type> write_time{};
};

class EditableShaderToyReloadController {
private:
    std::string m_shader_source_path{};
    bool m_auto_reload_enabled{true};
    bool m_reload_requested{true};
    std::optional<std::filesystem::path> m_last_compiled_path{};
    EditableShaderToyReloadState m_state{};

    static std::shared_ptr<spdlog::logger> logger() {
        return utils::get_logger("system.render.editable_shadertoy_reload");
    }

    static std::filesystem::path project_source_root() {
#ifdef RTR_PROJECT_SOURCE_DIR
        return std::filesystem::path(RTR_PROJECT_SOURCE_DIR);
#else
        return {};
#endif
    }

public:
    explicit EditableShaderToyReloadController(
        std::string shader_source_path = {},
        bool auto_reload_enabled = true
    )
        : m_shader_source_path(std::move(shader_source_path)),
          m_auto_reload_enabled(auto_reload_enabled) {}

    const std::string& shader_source_path() const { return m_shader_source_path; }
    bool auto_reload_enabled() const { return m_auto_reload_enabled; }
    const EditableShaderToyReloadState& reload_state() const { return m_state; }

    void set_shader_source_path(std::string shader_source_path) {
        if (m_shader_source_path == shader_source_path) {
            return;
        }
        m_shader_source_path = std::move(shader_source_path);
        m_reload_requested = true;
    }

    void set_auto_reload_enabled(bool enabled) { m_auto_reload_enabled = enabled; }
    void request_reload() { m_reload_requested = true; }

    static std::filesystem::path resolve_source_path(std::string_view shader_source_path) {
        if (shader_source_path.empty()) {
            return {};
        }

        const std::filesystem::path input(shader_source_path);
        if (input.is_absolute()) {
            return input.lexically_normal();
        }

        const std::filesystem::path source_root = project_source_root();
        if (source_root.empty()) {
            return input.lexically_normal();
        }
        return (source_root / input).lexically_normal();
    }

    EditableShaderToyReloadCheckResult check_for_reload() {
        EditableShaderToyReloadCheckResult result{};
        const std::filesystem::path resolved_path = resolve_source_path(m_shader_source_path);
        m_state.resolved_path = resolved_path;

        if (m_shader_source_path.empty()) {
            m_state.phase = EditableShaderToyReloadPhase::Error;
            m_state.last_write_time.reset();
            m_state.has_valid_program = false;
            m_state.last_error_message = "Shader source path is empty.";
            return result;
        }

        std::error_code ec;
        const bool exists = std::filesystem::exists(resolved_path, ec);
        if (ec || !exists) {
            m_state.phase = EditableShaderToyReloadPhase::MissingFile;
            m_state.last_write_time.reset();
            m_state.has_valid_program = false;
            m_state.last_error_message =
                ec ? ("Failed to query shader file: " + ec.message())
                   : ("Shader source file does not exist: " + resolved_path.string());
            return result;
        }

        const auto write_time = std::filesystem::last_write_time(resolved_path, ec);
        if (ec) {
            m_state.phase = EditableShaderToyReloadPhase::Error;
            m_state.last_write_time.reset();
            m_state.has_valid_program = false;
            m_state.last_error_message = "Failed to read shader file timestamp: " + ec.message();
            return result;
        }

        const bool path_changed =
            !m_last_compiled_path.has_value() || m_last_compiled_path.value() != resolved_path;
        const bool time_changed =
            !m_state.last_write_time.has_value() || m_state.last_write_time.value() != write_time;

        if (!m_reload_requested && !path_changed && !(m_auto_reload_enabled && time_changed)) {
            return result;
        }

        m_reload_requested = false;
        m_state.phase = EditableShaderToyReloadPhase::Compiling;
        result.should_compile = true;
        result.resolved_path = resolved_path;
        result.write_time = write_time;
        return result;
    }

    void apply_compile_success(
        const std::filesystem::path& resolved_path,
        std::optional<std::filesystem::file_time_type> write_time
    ) {
        m_last_compiled_path = resolved_path;
        m_state.phase = EditableShaderToyReloadPhase::Ready;
        m_state.resolved_path = resolved_path;
        m_state.last_write_time = write_time;
        m_state.last_error_message.clear();
        m_state.has_valid_program = true;
        logger()->info("Editable ShaderToy shader compiled successfully: {}", resolved_path.string());
    }

    void apply_compile_failure(
        const std::filesystem::path& resolved_path,
        std::optional<std::filesystem::file_time_type> write_time,
        std::string diagnostics,
        EditableShaderToyReloadPhase phase = EditableShaderToyReloadPhase::Error
    ) {
        m_state.phase = phase;
        m_state.resolved_path = resolved_path;
        m_state.last_write_time = write_time;
        m_state.last_error_message = std::move(diagnostics);
        m_state.has_valid_program = false;

        if (phase == EditableShaderToyReloadPhase::MissingFile) {
            logger()->error("Editable ShaderToy shader file missing: {}", resolved_path.string());
        } else {
            logger()->error("Editable ShaderToy shader compilation failed for '{}': {}",
                            resolved_path.string(), m_state.last_error_message);
        }
    }
};

inline SlangFileCompileResult compile_editable_shadertoy_shader(
    const std::filesystem::path& shader_source_path
) {
    return compile_slang_file_to_spirv(shader_source_path, vk::ShaderStageFlagBits::eCompute);
}

}  // namespace rtr::system::render
