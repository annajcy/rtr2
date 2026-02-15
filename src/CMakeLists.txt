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
    spdlog::spdlog
    TinyGLTF::TinyGLTF
    slang::slang
    tinyobjloader::tinyobjloader
    pugixml::pugixml
    pbpt::pbpt
)
