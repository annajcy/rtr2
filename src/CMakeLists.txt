add_library(rtr INTERFACE)
add_library(rtr::rtr ALIAS rtr)
target_include_directories(rtr INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external/pbpt/src>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)

target_link_libraries(rtr INTERFACE 
    imgui_vk 
    stb_impl 
    glfw 
    Vulkan::Loader 
  TinyGLTF::TinyGLTF
    slang::slang
    glm::glm
    tinyobjloader::tinyobjloader
    pugixml::pugixml
)

set(RTR_FRAMEWORK_INTEGRATION_SOURCE rtr/framework/integration/pbpt/pbpt_offline_render_service.cpp)
if(NOT RTR_ENABLE_PBPT_RUNTIME)
  set(RTR_FRAMEWORK_INTEGRATION_SOURCE rtr/framework/integration/pbpt/pbpt_offline_render_service_stub.cpp)
endif()

add_library(rtr_framework_integration STATIC
  ${RTR_FRAMEWORK_INTEGRATION_SOURCE}
)
  add_library(rtr::framework_integration ALIAS rtr_framework_integration)
target_include_directories(rtr_framework_integration
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
      $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    PRIVATE
      $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/external/pbpt/src>
)
target_link_libraries(rtr_framework_integration PUBLIC rtr)
if(RTR_ENABLE_PBPT_RUNTIME)
  target_link_libraries(rtr_framework_integration PRIVATE
    pbpt::pbpt
  )
endif()
