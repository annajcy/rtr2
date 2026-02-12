include(CTest)
include(GoogleTest)

add_executable(event_center_test
    event_center_test.cpp
)
target_include_directories(event_center_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(event_center_test PRIVATE gtest::gtest)
gtest_discover_tests(event_center_test)

add_executable(input_system_test
    input_system_test.cpp
)
target_include_directories(input_system_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(input_system_test PRIVATE gtest::gtest glfw Vulkan::Loader)
gtest_discover_tests(input_system_test)

add_executable(framework_engine_test
    framework_engine_test.cpp
)
target_include_directories(framework_engine_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_engine_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_engine_test)

add_executable(framework_world_scene_test
    framework_world_scene_test.cpp
)
target_include_directories(framework_world_scene_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_world_scene_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_world_scene_test)

add_executable(framework_scene_graph_test
    framework_scene_graph_test.cpp
)
target_include_directories(framework_scene_graph_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_scene_graph_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_scene_graph_test)

add_executable(framework_camera_test
    framework_camera_test.cpp
)
target_include_directories(framework_camera_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_camera_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_camera_test)

add_executable(framework_mesh_renderer_test
    framework_mesh_renderer_test.cpp
)
target_include_directories(framework_mesh_renderer_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_mesh_renderer_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_mesh_renderer_test)

add_executable(framework_pbpt_mesh_test
    framework_pbpt_mesh_test.cpp
)
target_include_directories(framework_pbpt_mesh_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_pbpt_mesh_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_pbpt_mesh_test)

add_executable(framework_pbpt_light_test
    framework_pbpt_light_test.cpp
)
target_include_directories(framework_pbpt_light_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_pbpt_light_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_pbpt_light_test)

add_executable(framework_forward_scene_view_builder_test
    framework_forward_scene_view_builder_test.cpp
)
target_include_directories(framework_forward_scene_view_builder_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_forward_scene_view_builder_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_forward_scene_view_builder_test)

add_executable(framework_pbpt_scene_export_builder_test
    framework_pbpt_scene_export_builder_test.cpp
)
target_include_directories(framework_pbpt_scene_export_builder_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(framework_pbpt_scene_export_builder_test PRIVATE gtest::gtest glm::glm)
gtest_discover_tests(framework_pbpt_scene_export_builder_test)

add_executable(framework_free_look_camera_controller_test
    framework_free_look_camera_controller_test.cpp
)
target_include_directories(framework_free_look_camera_controller_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(
    framework_free_look_camera_controller_test
    PRIVATE
        gtest::gtest
        glm::glm
        glfw
        Vulkan::Loader
)
gtest_discover_tests(framework_free_look_camera_controller_test)

add_executable(framework_trackball_camera_controller_test
    framework_trackball_camera_controller_test.cpp
)
target_include_directories(framework_trackball_camera_controller_test PRIVATE ${CMAKE_SOURCE_DIR}/src)
target_link_libraries(
    framework_trackball_camera_controller_test
    PRIVATE
        gtest::gtest
        glm::glm
        glfw
        Vulkan::Loader
)
gtest_discover_tests(framework_trackball_camera_controller_test)
