cmake_minimum_required(VERSION 3.27)
project(rtr LANGUAGES CXX)

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

option(RTR_BUILD_TESTS "Build the project's tests" ON)
option(RTR_BUILD_EXAMPLES "Build the project's examples" ON)
option(RTR_COMPILE_SHADERS "Build the project's shaders" ON)

include(third_party) 
add_subdirectory(src)

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
  add_subdirectory(tests)
  message(STATUS "RTR_BUILD_TESTS is ON; tests will be built.")
else()
  message(STATUS "RTR_BUILD_TESTS is OFF; tests will NOT be built.")
endif()



