add_library(rtr_runtime INTERFACE)
add_library(rtr::runtime ALIAS rtr_runtime)
set_target_properties(rtr_runtime PROPERTIES EXPORT_NAME runtime)

target_include_directories(rtr_runtime INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external/pbpt/src>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(rtr_runtime INTERFACE
  stb_impl
  Eigen3::Eigen
  glfw
  Vulkan::Loader
  spdlog::spdlog
  TinyGLTF::TinyGLTF
  slang::slang
  tinyobjloader::tinyobjloader
  pbpt::pbpt
)

target_compile_definitions(rtr_runtime INTERFACE
  RTR_DEFAULT_SHADER_OUTPUT_DIR=\"${RTR_DEFAULT_SHADER_OUTPUT_DIR}\"
  RTR_PROJECT_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}\"
)

add_executable(rtr_mcp_bridge mcp_bridge_main.cpp)
target_link_libraries(rtr_mcp_bridge PRIVATE
  rtr::runtime
  nlohmann_json::nlohmann_json
)
target_compile_definitions(rtr_mcp_bridge PRIVATE
  RTR_MCP_BRIDGE_VERSION=\"0.1.0\"
)

if (RTR_BUILD_EDITOR)
  add_library(rtr_editor INTERFACE)
  add_library(rtr::editor ALIAS rtr_editor)
  set_target_properties(rtr_editor PROPERTIES EXPORT_NAME editor)
  target_include_directories(rtr_editor INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external/pbpt/src>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  )
  target_link_libraries(rtr_editor INTERFACE
    rtr_runtime
    imgui_vk
    imguizmo::imguizmo
  )
endif()
