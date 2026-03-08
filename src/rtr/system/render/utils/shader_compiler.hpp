#pragma once

#include <cstring>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "slang.h"
#include "vulkan/vulkan.hpp"

namespace rtr::system::render {

struct SlangFileCompileResult {
    bool ok{false};
    std::filesystem::path source_path{};
    std::optional<std::filesystem::file_time_type> last_write_time{};
    std::vector<char> spirv_code{};
    std::string diagnostics{};
    std::string entry_point{"main"};
};

inline SlangFileCompileResult compile_slang_file_to_spirv(
    const std::filesystem::path& shader_source_path,
    vk::ShaderStageFlagBits expected_stage
) {
    SlangFileCompileResult result{};
    result.source_path = shader_source_path;

    std::error_code ec;
    if (!std::filesystem::exists(shader_source_path, ec)) {
        result.diagnostics =
            ec ? ("Failed to query shader file: " + ec.message())
               : ("Shader source file does not exist: " + shader_source_path.string());
        return result;
    }

    result.last_write_time = std::filesystem::last_write_time(shader_source_path, ec);
    if (ec) {
        result.diagnostics = "Failed to read shader file timestamp: " + ec.message();
        return result;
    }

    SlangSession* const global_session = spCreateSession(nullptr);
    if (global_session == nullptr) {
        result.diagnostics = "Failed to create Slang session.";
        return result;
    }

    SlangCompileRequest* const request = spCreateCompileRequest(global_session);
    if (request == nullptr) {
        result.diagnostics = "Failed to create Slang compile request.";
        spDestroySession(global_session);
        return result;
    }

    auto cleanup = [&]() {
        spDestroyCompileRequest(request);
        spDestroySession(global_session);
    };

    const int target_index = spAddCodeGenTarget(request, SLANG_SPIRV);
    spSetTargetProfile(request, target_index, spFindProfile(global_session, "spirv_1_5"));

    const int translation_unit_index = spAddTranslationUnit(request, SLANG_SOURCE_LANGUAGE_SLANG, nullptr);
    spAddTranslationUnitSourceFile(request, translation_unit_index, shader_source_path.string().c_str());

    const SlangResult compile_result = spCompile(request);

    if (const char* diagnostics = spGetDiagnosticOutput(request); diagnostics != nullptr) {
        result.diagnostics = diagnostics;
    }

    if (SLANG_FAILED(compile_result)) {
        cleanup();
        return result;
    }

    auto stage_matches = [](SlangStage stage, vk::ShaderStageFlagBits expected) {
        switch (expected) {
        case vk::ShaderStageFlagBits::eVertex:
            return stage == SLANG_STAGE_VERTEX;
        case vk::ShaderStageFlagBits::eFragment:
            return stage == SLANG_STAGE_FRAGMENT;
        case vk::ShaderStageFlagBits::eCompute:
            return stage == SLANG_STAGE_COMPUTE;
        default:
            return false;
        }
    };

    auto stage_label = [](vk::ShaderStageFlagBits expected) -> const char* {
        switch (expected) {
        case vk::ShaderStageFlagBits::eVertex:
            return "vertex";
        case vk::ShaderStageFlagBits::eFragment:
            return "fragment";
        case vk::ShaderStageFlagBits::eCompute:
            return "compute";
        default:
            return "requested";
        }
    };

    SlangReflection* const reflection = spGetReflection(request);
    const SlangUInt entry_point_count = reflection ? spReflection_getEntryPointCount(reflection) : 0;
    std::optional<SlangUInt> matched_entry_index{};

    for (SlangUInt entry_index = 0; entry_index < entry_point_count; ++entry_index) {
        SlangReflectionEntryPoint* const entry_point =
            spReflection_getEntryPointByIndex(reflection, entry_index);
        if (entry_point == nullptr) {
            continue;
        }
        if (!stage_matches(spReflectionEntryPoint_getStage(entry_point), expected_stage)) {
            continue;
        }
        if (matched_entry_index.has_value()) {
            result.diagnostics =
                std::string("Shader source must contain exactly one ") + stage_label(expected_stage) + " entry point.";
            cleanup();
            return result;
        }
        matched_entry_index = entry_index;
    }

    if (!matched_entry_index.has_value()) {
        result.diagnostics =
            std::string("Shader source must contain one ") + stage_label(expected_stage) + " entry point.";
        cleanup();
        return result;
    }

    size_t code_size = 0;
    const void* code = spGetEntryPointCode(request, matched_entry_index.value(), &code_size);
    if (code == nullptr || code_size == 0) {
        result.diagnostics = "Failed to extract compiled SPIR-V for selected entry point.";
        cleanup();
        return result;
    }

    result.spirv_code.resize(code_size);
    std::memcpy(result.spirv_code.data(), code, code_size);
    // Slang emits a single-entry SPIR-V module for spGetEntryPointCode(...),
    // and Vulkan expects the generated module entry name rather than the
    // source-language symbol. In practice this entry is "main".
    result.entry_point = "main";
    result.ok = true;

    cleanup();
    return result;
}

}  // namespace rtr::system::render
