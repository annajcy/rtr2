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
  )
endif()
