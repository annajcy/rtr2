if (TARGET rtr_editor)
  add_executable(quickstart quickstart.cpp)
  target_link_libraries(quickstart PRIVATE rtr::runtime rtr::editor)

  add_executable(offline_cbox offline_cbox.cpp)
  target_link_libraries(offline_cbox PRIVATE rtr::runtime rtr::editor)
endif()

add_executable(shadertoy shadertoy.cpp)
target_link_libraries(shadertoy PRIVATE rtr::runtime)