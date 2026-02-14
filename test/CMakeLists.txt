include(CTest)
include(GoogleTest)

function(rtr_add_test TARGET_NAME TEST_SOURCE)
    add_executable(${TARGET_NAME} ${TEST_SOURCE})
    target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(${TARGET_NAME} PRIVATE gtest::gtest ${ARGN})
    gtest_discover_tests(
        ${TARGET_NAME}
        DISCOVERY_TIMEOUT 60
        PROPERTIES LABELS "unit"
    )
endfunction()

function(rtr_add_integration_test TARGET_NAME TEST_SOURCE)
    add_executable(${TARGET_NAME} ${TEST_SOURCE})
    target_include_directories(${TARGET_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(${TARGET_NAME} PRIVATE gtest::gtest ${ARGN})
    gtest_discover_tests(
        ${TARGET_NAME}
        DISCOVERY_TIMEOUT 60
        PROPERTIES LABELS "integration;gpu"
    )
endfunction()

# utils
rtr_add_test(
    test_event_center
    utils/event_center_test.cpp
)
rtr_add_test(
    test_file_loader
    utils/file_loader_test.cpp
)
rtr_add_test(
    test_obj_loader
    utils/obj_loader_test.cpp
    glm::glm
    tinyobjloader::tinyobjloader
)

# system
rtr_add_test(
    test_input_system
    system/input/input_system_test.cpp
    glfw
    Vulkan::Loader
)
rtr_add_test(
    test_framework_forward_scene_view_builder
    system/render/framework_forward_scene_view_builder_test.cpp
    glm::glm
)
rtr_add_test(
    test_pipeline_base
    system/render/pipeline_base_test.cpp
    rtr
)
rtr_add_integration_test(
    test_frame_scheduler_integration
    system/render/frame_scheduler_integration_test.cpp
    rtr
)
rtr_add_integration_test(
    test_renderer_integration
    system/render/renderer_integration_test.cpp
    rtr
)

# framework/core
rtr_add_test(
    test_framework_engine
    framework/core/framework_engine_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_world_scene
    framework/core/framework_world_scene_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_scene_graph
    framework/core/framework_scene_graph_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_camera
    framework/core/framework_camera_test.cpp
    glm::glm
)

# framework/component
rtr_add_test(
    test_framework_mesh_renderer
    framework/component/framework_mesh_renderer_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_pbpt_mesh
    framework/component/framework_pbpt_mesh_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_pbpt_light
    framework/component/framework_pbpt_light_test.cpp
    glm::glm
)
rtr_add_test(
    test_pbpt_spectrum
    framework/component/pbpt_spectrum_test.cpp
)
rtr_add_test(
    test_framework_free_look_camera_controller
    framework/component/framework_free_look_camera_controller_test.cpp
    glm::glm
    glfw
    Vulkan::Loader
)
rtr_add_test(
    test_framework_trackball_camera_controller
    framework/component/framework_trackball_camera_controller_test.cpp
    glm::glm
    glfw
    Vulkan::Loader
)

# framework/integration
rtr_add_test(
    test_framework_pbpt_scene_export_builder
    framework/integration/framework_pbpt_scene_export_builder_test.cpp
    glm::glm
)
rtr_add_test(
    test_framework_pbpt_scene_importer
    framework/integration/framework_pbpt_scene_importer_test.cpp
    rtr
)

# rhi
rtr_add_test(
    test_image_transition
    rhi/image_transition_test.cpp
    rtr
)

if(RTR_ENABLE_PBPT_RUNTIME)
    rtr_add_test(
        test_framework_pbpt_offline_render_service
        framework/integration/framework_pbpt_offline_render_service_test.cpp
        rtr_framework_integration
        rtr
    )
else()
    rtr_add_test(
        test_framework_pbpt_offline_render_service_stub
        framework/integration/framework_pbpt_offline_render_service_stub_test.cpp
        rtr_framework_integration
        rtr
    )
endif()
