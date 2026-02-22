if (TARGET rtr_editor)
  add_executable(quickstart editor/quickstart.cpp)
  target_link_libraries(quickstart PRIVATE rtr::runtime rtr::editor)

  add_executable(offline_cbox editor/offline_cbox.cpp)
  target_link_libraries(offline_cbox PRIVATE rtr::runtime rtr::editor)

  add_executable(shadertoy_editor editor/shadertoy_editor.cpp)
  target_link_libraries(shadertoy_editor PRIVATE rtr::runtime rtr::editor)
endif()

add_executable(shadertoy runtime/shadertoy.cpp)
target_link_libraries(shadertoy PRIVATE rtr::runtime)