cmake_minimum_required(VERSION 3.15)
project(rtr_test_package LANGUAGES CXX)

find_package(rtr CONFIG REQUIRED COMPONENTS rtr framework_integration)

add_executable(test_package test_package.cpp)
target_compile_features(test_package PRIVATE cxx_std_23)
target_link_libraries(test_package PRIVATE rtr::rtr rtr::framework_integration)
