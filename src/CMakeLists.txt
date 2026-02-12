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

set(PBPT_EMBED_ROOT "${CMAKE_SOURCE_DIR}/external/pbpt")
set(PBPT_EMBED_SRC_DIR "${PBPT_EMBED_ROOT}/src")

find_package(Python3 COMPONENTS Interpreter REQUIRED)

set(PBPT_SRGB_SPECTRUM_INPUT "${PBPT_EMBED_ROOT}/asset/rgb2spec/rgbspectrum_srgb.cpp")
set(PBPT_SRGB_SPECTRUM_SCRIPT "${PBPT_EMBED_ROOT}/script/generate_rgb_spectrum_lut.py")
set(PBPT_SRGB_SPECTRUM_OUTPUT "${CMAKE_BINARY_DIR}/generated/pbpt/srgb_spectrum_lut_data.cpp")

add_custom_command(
    OUTPUT "${PBPT_SRGB_SPECTRUM_OUTPUT}"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_BINARY_DIR}/generated/pbpt"
    COMMAND ${Python3_EXECUTABLE} "${PBPT_SRGB_SPECTRUM_SCRIPT}"
            --input "${PBPT_SRGB_SPECTRUM_INPUT}"
            --output "${PBPT_SRGB_SPECTRUM_OUTPUT}"
    DEPENDS "${PBPT_SRGB_SPECTRUM_INPUT}" "${PBPT_SRGB_SPECTRUM_SCRIPT}"
    COMMENT "Generating PBPT sRGB->spectrum LUT translation unit"
    VERBATIM
)
set_source_files_properties("${PBPT_SRGB_SPECTRUM_OUTPUT}" PROPERTIES GENERATED TRUE)

add_custom_target(generate_pbpt_rgb_spectrum_lut DEPENDS "${PBPT_SRGB_SPECTRUM_OUTPUT}")

add_library(pbpt_rgb_spectrum_lut STATIC "${PBPT_SRGB_SPECTRUM_OUTPUT}")
add_dependencies(pbpt_rgb_spectrum_lut generate_pbpt_rgb_spectrum_lut)
target_include_directories(pbpt_rgb_spectrum_lut PRIVATE "${PBPT_EMBED_SRC_DIR}")

add_library(pbpt_embed INTERFACE)
target_include_directories(pbpt_embed INTERFACE "${PBPT_EMBED_SRC_DIR}")
target_link_libraries(pbpt_embed INTERFACE
    pbpt_rgb_spectrum_lut
    assimp::assimp
    OpenEXR::OpenEXR
    pugixml::pugixml
    embree::embree
    TBB::tbb
)

add_library(rtr_framework_integration STATIC
    framework/integration/pbpt_offline_render_service.cpp
)
target_include_directories(rtr_framework_integration
    PRIVATE BEFORE "${PBPT_EMBED_SRC_DIR}"
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(rtr_framework_integration PUBLIC
    rtr
    pbpt_rgb_spectrum_lut
    assimp::assimp
    OpenEXR::OpenEXR
    pugixml::pugixml
    embree::embree
    TBB::tbb
)
