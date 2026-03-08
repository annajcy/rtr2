cmake_minimum_required(VERSION 3.15)
project(imguizmo LANGUAGES CXX)

find_package(imgui CONFIG REQUIRED)

add_library(imguizmo STATIC ImGuizmo.cpp)
add_library(imguizmo::imguizmo ALIAS imguizmo)

target_include_directories(imguizmo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(imguizmo PUBLIC imgui::imgui)

set_target_properties(imguizmo PROPERTIES
  POSITION_INDEPENDENT_CODE ON
)

install(TARGETS imguizmo
  EXPORT imguizmoTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/ImGuizmo.h
  DESTINATION include
)
