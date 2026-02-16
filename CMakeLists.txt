cmake_minimum_required(VERSION 3.27)
project(rtr VERSION 0.1.0 LANGUAGES CXX)

message(STATUS "CMake version: ${CMAKE_VERSION}")

option(ENABLE_COMPILE_COMMANDS "Copy compile_commands.json to build root" ON)
if (ENABLE_COMPILE_COMMANDS)
  set(_SRC "${CMAKE_CURRENT_BINARY_DIR}/compile_commands.json")
  set(_DST "${CMAKE_SOURCE_DIR}/build/compile_commands.json")

  add_custom_target(copy_compile_commands ALL
    COMMAND ${CMAKE_COMMAND} -E copy_if_different "${_SRC}" "${_DST}"
    COMMENT "Copying compile_commands.json -> ${_DST}"
    VERBATIM
  )

  message(STATUS "compile_commands.json will be copied to build root.")
else()
  message(STATUS "compile_commands.json will NOT be copied to build root.")
endif()

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Suppress duplicate library warnings on macOS
if(APPLE)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-no_warn_duplicate_libraries")
endif()

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

set(RTR_DEFAULT_BUILD_TESTS ${PROJECT_IS_TOP_LEVEL})
set(RTR_DEFAULT_BUILD_EXAMPLES ${PROJECT_IS_TOP_LEVEL})
set(RTR_DEFAULT_BUILD_EDITOR ON)

option(RTR_BUILD_TESTS "Build the project's tests" ${RTR_DEFAULT_BUILD_TESTS})
option(RTR_BUILD_EXAMPLES "Build the project's examples" ${RTR_DEFAULT_BUILD_EXAMPLES})
option(RTR_BUILD_EDITOR "Build editor module (ImGui integration)" ${RTR_DEFAULT_BUILD_EDITOR})
option(RTR_COMPILE_SHADERS "Build the project's shaders" ON)

set(PBPT_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(PBPT_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(PBPT_BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
add_subdirectory(external/pbpt)

include(third_party) 
add_subdirectory(src)

set(RTR_CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/rtr")
set(RTR_CONFIG_BUILD_EDITOR ${RTR_BUILD_EDITOR})

set(RTR_INSTALL_TARGETS rtr_runtime stb_impl)
if (TARGET rtr_editor)
  list(APPEND RTR_INSTALL_TARGETS rtr_editor)
endif()
if (TARGET imgui_vk)
  list(APPEND RTR_INSTALL_TARGETS imgui_vk)
endif()

install(
  TARGETS ${RTR_INSTALL_TARGETS}
  EXPORT rtrTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/src/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/pbpt/src/pbpt/
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/pbpt
  FILES_MATCHING
  PATTERN "*.hpp"
  PATTERN "*.h"
)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/rtrConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/rtrConfig.cmake
  INSTALL_DESTINATION ${RTR_CONFIG_INSTALL_DIR}
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/rtrConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/rtrConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/rtrConfigVersion.cmake
  DESTINATION ${RTR_CONFIG_INSTALL_DIR}
)

install(
  EXPORT rtrTargets
  NAMESPACE rtr::
  DESTINATION ${RTR_CONFIG_INSTALL_DIR}
)

if (RTR_COMPILE_SHADERS)
  add_subdirectory(shaders)
  message(STATUS "RTR_COMPILE_SHADERS is ON; shaders will be compiled.")
else()
  message(STATUS "RTR_COMPILE_SHADERS is OFF; shaders will NOT be compiled.")
endif()

if (RTR_BUILD_EXAMPLES)
  add_subdirectory(examples)
  message(STATUS "RTR_BUILD_EXAMPLES is ON; examples will be built.")
else()
  message(STATUS "RTR_BUILD_EXAMPLES is OFF; examples will NOT be built.")
endif()

if(RTR_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test)
  message(STATUS "RTR_BUILD_TESTS is ON; tests will be built.")
else()
  message(STATUS "RTR_BUILD_TESTS is OFF; tests will NOT be built.")
endif()
