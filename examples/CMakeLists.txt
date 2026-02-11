add_executable(main main.cpp)
target_link_libraries(main PRIVATE rtr)

add_executable(third_party third_party.cpp)
target_link_libraries(third_party PRIVATE rtr)

add_executable(shadertoy_main shadertoy_main.cpp)
target_link_libraries(shadertoy_main PRIVATE rtr)

add_executable(compute_buffer_add_main compute_buffer_add_main.cpp)
target_link_libraries(compute_buffer_add_main PRIVATE rtr)
