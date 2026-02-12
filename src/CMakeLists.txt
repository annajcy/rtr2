add_library(rtr INTERFACE)
target_include_directories(rtr INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})

set(_PUGIXML_INC_DIR "")
if (CMAKE_BUILD_TYPE STREQUAL "Debug" AND DEFINED pugixml_PACKAGE_FOLDER_DEBUG)
  set(_PUGIXML_INC_DIR "${pugixml_PACKAGE_FOLDER_DEBUG}/include")
elseif (CMAKE_BUILD_TYPE STREQUAL "Release" AND DEFINED pugixml_PACKAGE_FOLDER_RELEASE)
  set(_PUGIXML_INC_DIR "${pugixml_PACKAGE_FOLDER_RELEASE}/include")
elseif (CMAKE_BUILD_TYPE STREQUAL "RelWithDebInfo" AND DEFINED pugixml_PACKAGE_FOLDER_RELWITHDEBINFO)
  set(_PUGIXML_INC_DIR "${pugixml_PACKAGE_FOLDER_RELWITHDEBINFO}/include")
elseif (CMAKE_BUILD_TYPE STREQUAL "MinSizeRel" AND DEFINED pugixml_PACKAGE_FOLDER_MINSIZEREL)
  set(_PUGIXML_INC_DIR "${pugixml_PACKAGE_FOLDER_MINSIZEREL}/include")
endif()

if (NOT _PUGIXML_INC_DIR STREQUAL "")
  target_include_directories(rtr INTERFACE "${_PUGIXML_INC_DIR}")
endif()

target_link_libraries(rtr INTERFACE 
    imgui_vk 
    stb_impl 
    glfw 
    Vulkan::Loader 
    assimp::assimp 
    slang::slang
    glm::glm
    tinyobjloader::tinyobjloader
    pugixml::pugixml
)
