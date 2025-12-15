add_executable(compile_shaders compile_shaders.cpp)
find_package(slang REQUIRED)
target_link_libraries(compile_shaders PRIVATE slang::slang)

# 定义 slang 源文件目录和输出目录
set(SLANG_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(SLANG_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}/compiled")

# 添加自定义目标，在构建时运行 compile_shaders
add_custom_target(compile_slang_shaders ALL
    COMMAND compile_shaders "${SLANG_SOURCE_DIR}" "${SLANG_OUTPUT_DIR}"
    DEPENDS compile_shaders
    COMMENT "Compiling all .slang files to SPIR-V"
    VERBATIM
)

# 导出编译后的 SPIR-V 文件路径供其他目标使用
set(SLANG_SPIRV_DIR "${SLANG_OUTPUT_DIR}" PARENT_SCOPE)