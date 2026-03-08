#include <chrono>
#include <filesystem>
#include <fstream>
#include <thread>

#include "gtest/gtest.h"

#include "rtr/system/render/pipeline/shadertoy/editable_shadertoy_reload_controller.hpp"
#include "rtr/system/render/utils/shader_compiler.hpp"

namespace rtr::system::render::test {

namespace {

class TempShaderFile {
private:
    std::filesystem::path m_path;

public:
    explicit TempShaderFile(std::string_view filename)
        : m_path(std::filesystem::temp_directory_path() / filename) {}

    ~TempShaderFile() {
        std::error_code ec;
        std::filesystem::remove(m_path, ec);
    }

    const std::filesystem::path& path() const { return m_path; }

    void write(std::string_view content) const {
        std::ofstream out(m_path, std::ios::binary | std::ios::trunc);
        out << content;
        out.close();
    }

    void bump_timestamp() const {
        std::error_code ec;
        const auto current_time = std::filesystem::last_write_time(m_path, ec);
        ASSERT_FALSE(ec);
        std::filesystem::last_write_time(m_path, current_time + std::chrono::seconds(1), ec);
        ASSERT_FALSE(ec);
    }
};

constexpr std::string_view kValidComputeShader = R"(
struct ShaderToyParams {
    float4 iResolution;
    float4 iTime;
    float4 iParams;
};

[[vk_binding(0, 0)]]
ConstantBuffer<ShaderToyParams> params;

[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    outColor[tid.xy] = float4(1.0, 0.0, 0.0, 1.0);
}
)";

constexpr std::string_view kNoEntryPointShader = R"(
float4 value() {
    return float4(1.0, 0.0, 0.0, 1.0);
}
)";

constexpr std::string_view kTwoComputeEntryPointsShader = R"(
[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

[numthreads(8, 8, 1)]
[shader("compute")]
void compA(uint3 tid : SV_DispatchThreadID) {
    outColor[tid.xy] = float4(1.0, 0.0, 0.0, 1.0);
}

[numthreads(8, 8, 1)]
[shader("compute")]
void compB(uint3 tid : SV_DispatchThreadID) {
    outColor[tid.xy] = float4(0.0, 1.0, 0.0, 1.0);
}
)";

constexpr std::string_view kSyntaxErrorShader = R"(
[[vk_binding(1, 0)]]
RWTexture2D<float4> outColor;

[numthreads(8, 8, 1)]
[shader("compute")]
void compMain(uint3 tid : SV_DispatchThreadID) {
    outColor[tid.xy] = float4(1.0, 0.0, 0.0, 1.0)
}
)";

}  // namespace

TEST(EditableShaderToyShaderCompilerTest, ResolvesRelativeAndAbsoluteSourcePaths) {
    const auto absolute = std::filesystem::temp_directory_path() / "editable_shadertoy_abs_test.slang";
    EXPECT_EQ(
        EditableShaderToyReloadController::resolve_source_path(absolute.string()),
        absolute.lexically_normal()
    );

    const auto relative = EditableShaderToyReloadController::resolve_source_path("shaders/editable_shadertoy_compute.slang");
    EXPECT_TRUE(relative.is_absolute());
    EXPECT_EQ(relative.filename(), "editable_shadertoy_compute.slang");
}

TEST(EditableShaderToyShaderCompilerTest, ControllerTracksInitialCompileReloadAndFailures) {
    TempShaderFile shader_file("editable_shadertoy_controller_test.slang");
    shader_file.write(kValidComputeShader);

    EditableShaderToyReloadController controller(shader_file.path().string(), true);

    const auto first_check = controller.check_for_reload();
    EXPECT_TRUE(first_check.should_compile);
    EXPECT_EQ(first_check.resolved_path, shader_file.path());
    ASSERT_TRUE(first_check.write_time.has_value());

    controller.apply_compile_success(first_check.resolved_path, first_check.write_time);
    EXPECT_TRUE(controller.reload_state().has_valid_program);
    EXPECT_EQ(controller.reload_state().phase, EditableShaderToyReloadPhase::Ready);

    const auto steady_check = controller.check_for_reload();
    EXPECT_FALSE(steady_check.should_compile);

    shader_file.bump_timestamp();
    const auto modified_check = controller.check_for_reload();
    EXPECT_TRUE(modified_check.should_compile);
    ASSERT_TRUE(modified_check.write_time.has_value());
    EXPECT_NE(modified_check.write_time, first_check.write_time);

    controller.apply_compile_failure(
        modified_check.resolved_path,
        modified_check.write_time,
        "synthetic compile failure"
    );
    EXPECT_FALSE(controller.reload_state().has_valid_program);
    EXPECT_EQ(controller.reload_state().phase, EditableShaderToyReloadPhase::Error);

    controller.request_reload();
    const auto manual_reload = controller.check_for_reload();
    EXPECT_TRUE(manual_reload.should_compile);
}

TEST(EditableShaderToyShaderCompilerTest, ControllerReportsMissingFilesAndEmptyPaths) {
    EditableShaderToyReloadController empty_controller("", true);
    const auto empty_check = empty_controller.check_for_reload();
    EXPECT_FALSE(empty_check.should_compile);
    EXPECT_EQ(empty_controller.reload_state().phase, EditableShaderToyReloadPhase::Error);

    EditableShaderToyReloadController missing_controller(
        (std::filesystem::temp_directory_path() / "editable_shadertoy_missing_test.slang").string(),
        true
    );
    const auto missing_check = missing_controller.check_for_reload();
    EXPECT_FALSE(missing_check.should_compile);
    EXPECT_EQ(missing_controller.reload_state().phase, EditableShaderToyReloadPhase::MissingFile);
}

TEST(EditableShaderToyShaderCompilerTest, CompilesValidComputeShader) {
    TempShaderFile shader_file("editable_shadertoy_compile_success_test.slang");
    shader_file.write(kValidComputeShader);

    const auto result = compile_editable_shadertoy_shader(shader_file.path());
    EXPECT_TRUE(result.ok);
    EXPECT_FALSE(result.spirv_code.empty());
    EXPECT_EQ(result.entry_point, "main");
}

TEST(EditableShaderToyShaderCompilerTest, RejectsShaderWithoutComputeEntryPoint) {
    TempShaderFile shader_file("editable_shadertoy_no_entry_test.slang");
    shader_file.write(kNoEntryPointShader);

    const auto result = compile_editable_shadertoy_shader(shader_file.path());
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.diagnostics.find("compute entry point"), std::string::npos);
}

TEST(EditableShaderToyShaderCompilerTest, RejectsShaderWithMultipleComputeEntryPoints) {
    TempShaderFile shader_file("editable_shadertoy_multiple_entry_test.slang");
    shader_file.write(kTwoComputeEntryPointsShader);

    const auto result = compile_editable_shadertoy_shader(shader_file.path());
    EXPECT_FALSE(result.ok);
    EXPECT_NE(result.diagnostics.find("exactly one compute entry point"), std::string::npos);
}

TEST(EditableShaderToyShaderCompilerTest, ReturnsDiagnosticsForSyntaxErrors) {
    TempShaderFile shader_file("editable_shadertoy_syntax_error_test.slang");
    shader_file.write(kSyntaxErrorShader);

    const auto result = compile_editable_shadertoy_shader(shader_file.path());
    EXPECT_FALSE(result.ok);
    EXPECT_FALSE(result.diagnostics.empty());
}

}  // namespace rtr::system::render::test

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
