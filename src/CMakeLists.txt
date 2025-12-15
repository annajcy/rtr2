add_library(rtr INTERFACE)
target_include_directories(rtr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(rtr INTERFACE 
    imgui_vk 
    stb_impl 
    glfw 
    Vulkan::Loader 
    assimp::assimp 
    slang::slang
    glm::glm
)