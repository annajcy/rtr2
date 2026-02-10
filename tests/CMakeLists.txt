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
