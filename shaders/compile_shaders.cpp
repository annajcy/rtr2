#include <cctype>
#include <fstream>
#include <iostream>
#include <filesystem>
#include <string>

#include "slang.h"

namespace fs = std::filesystem;

// 编译单个 slang 文件到 SPIR-V
bool compile_slang_file(const fs::path& input_path, const fs::path& output_path) {
    std::cout << "Compiling: " << input_path << " -> " << output_path << std::endl;
    
    // 创建 Slang global session
    SlangSession* global_session = spCreateSession(nullptr);
    if (!global_session) {
        std::cerr << "Failed to create Slang session" << std::endl;
        return false;
    }
    
    // 创建编译请求
    SlangCompileRequest* request = spCreateCompileRequest(global_session);
    
    // 设置代码生成目标为 SPIR-V
    int target_index = spAddCodeGenTarget(request, SLANG_SPIRV);
    spSetTargetProfile(request, target_index, spFindProfile(global_session, "spirv_1_5"));
    
    // 添加翻译单元
    int translation_unit_index = spAddTranslationUnit(request, SLANG_SOURCE_LANGUAGE_SLANG, nullptr);
    spAddTranslationUnitSourceFile(request, translation_unit_index, input_path.string().c_str());
    
    // 编译
    const SlangResult compile_result = spCompile(request);
    
    // 输出诊断信息
    if (auto diagnostics = spGetDiagnosticOutput(request)) {
        std::cout << diagnostics;
    }
    
    if (SLANG_FAILED(compile_result)) {
        std::cerr << "Failed to compile: " << input_path << std::endl;
        spDestroyCompileRequest(request);
        spDestroySession(global_session);
        return false;
    }
    
    SlangReflection* reflection = spGetReflection(request);
    SlangUInt entry_point_count = reflection ? spReflection_getEntryPointCount(reflection) : 0;

    if (entry_point_count == 0) {
        std::cerr << "No entry points found in: " << input_path << std::endl;
        spDestroyCompileRequest(request);
        spDestroySession(global_session);
        return false;
    }

    auto sanitizeSuffix = [](const std::string& raw) {
        std::string result;
        result.reserve(raw.size());
        for (char ch : raw) {
            if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_') {
                result.push_back(ch);
            } else {
                result.push_back('_');
            }
        }
        return result;
    };

    auto stageSuffix = [](SlangStage stage) -> std::string {
        switch (stage) {
            case SLANG_STAGE_VERTEX: return "vert";
            case SLANG_STAGE_FRAGMENT: return "frag";
            case SLANG_STAGE_COMPUTE: return "comp";
            default: return "";
        }
    };

    for (SlangUInt entry_index = 0; entry_index < entry_point_count; ++entry_index) {
        size_t code_size = 0;
        const void* code = spGetEntryPointCode(request, entry_index, &code_size);

        if (!code || code_size == 0) {
            std::cerr << "Failed to get compiled code for entry index " << entry_index
                      << " in: " << input_path << std::endl;
            spDestroyCompileRequest(request);
            spDestroySession(global_session);
            return false;
        }

        SlangReflectionEntryPoint* entry_point = spReflection_getEntryPointByIndex(reflection, entry_index);
        const char* entry_name_cstr = entry_point ? spReflectionEntryPoint_getName(entry_point) : nullptr;
        SlangStage stage = entry_point ? spReflectionEntryPoint_getStage(entry_point) : SLANG_STAGE_NONE;

        std::string entry_suffix = stageSuffix(stage);
        if (entry_suffix.empty() && entry_name_cstr) {
            entry_suffix = sanitizeSuffix(entry_name_cstr);
        }
        if (entry_suffix.empty()) {
            entry_suffix = "entry" + std::to_string(entry_index);
        }

        fs::path entry_output_path = output_path;
        fs::path parent_dir = entry_output_path.parent_path();
        std::string entry_filename = entry_output_path.stem().string() + "_" + entry_suffix + entry_output_path.extension().string();
        entry_output_path = parent_dir / entry_filename;

        fs::create_directories(parent_dir);
        std::ofstream outFile(entry_output_path, std::ios::binary);
        if (!outFile) {
            std::cerr << "Failed to open output file: " << entry_output_path << std::endl;
            spDestroyCompileRequest(request);
            spDestroySession(global_session);
            return false;
        }

        outFile.write(static_cast<const char*>(code), code_size);
        outFile.close();

        std::cout << "  -> Wrote entry '" << (entry_name_cstr ? entry_name_cstr : "(unnamed)")
                  << "' (" << entry_suffix << ") to " << entry_output_path << std::endl;
    }
    
    // 清理
    spDestroyCompileRequest(request);
    spDestroySession(global_session);
    
    std::cout << "Successfully compiled: " << input_path << std::endl;
    return true;
}

// 递归搜索并编译所有 .slang 文件
void compile_all_slang_files(const fs::path& source_dir, const fs::path& output_dir) {
    if (!fs::exists(output_dir)) {
        fs::create_directories(output_dir);
    }
    
    int success_count = 0;
    int fail_count = 0;
    
    for (const auto& entry : fs::recursive_directory_iterator(source_dir)) {
        if (entry.is_regular_file() && entry.path().extension() == ".slang") {
            // 计算相对路径并创建输出路径
            fs::path relative_path = fs::relative(entry.path(), source_dir);
            fs::path output_path = output_dir / relative_path;
            output_path.replace_extension(".spv");
            
            // 确保输出目录存在
            fs::create_directories(output_path.parent_path());
            
            // 编译文件
            if (compile_slang_file(entry.path(), output_path)) {
                success_count++;
            } else {
                fail_count++;
            }
        }
    }
    
    std::cout << "\n=== Compilation Summary ===" << std::endl;
    std::cout << "Successfully compiled: " << success_count << " files" << std::endl;
    std::cout << "Failed: " << fail_count << " files" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <slang_source_dir> [output_dir]" << std::endl;
        return 1;
    }
    
    fs::path source_dir = argv[1];
    fs::path output_dir = (argc >= 3) ? fs::path(argv[2]) : source_dir / "compiled";
    
    if (!fs::exists(source_dir)) {
        std::cerr << "Source directory does not exist: " << source_dir << std::endl;
        return 1;
    }
    
    std::cout << "Compiling Slang files from: " << source_dir << std::endl;
    std::cout << "Output directory: " << output_dir << std::endl;
    std::cout << "================================" << std::endl;
    
    compile_all_slang_files(source_dir, output_dir);
    
    return 0;
}
