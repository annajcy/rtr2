if (TARGET rtr_editor)
  add_executable(quickstart editor/quickstart.cpp)
  target_link_libraries(quickstart PRIVATE rtr::runtime rtr::editor)

  add_executable(offline_cbox editor/offline_cbox.cpp)
  target_link_libraries(offline_cbox PRIVATE rtr::runtime rtr::editor)

  add_executable(shadertoy_editor editor/shadertoy_editor.cpp)
  target_link_libraries(shadertoy_editor PRIVATE rtr::runtime rtr::editor)

  add_executable(gravity_reset_editor editor/gravity_reset_editor.cpp)
  target_link_libraries(gravity_reset_editor PRIVATE rtr::runtime rtr::editor)

  add_executable(gravity_spin_editor editor/gravity_spin_editor.cpp)
  target_link_libraries(gravity_spin_editor PRIVATE rtr::runtime rtr::editor)

  add_executable(sphere_box_collision_editor editor/sphere_box_collision_editor.cpp)
  target_link_libraries(sphere_box_collision_editor PRIVATE rtr::runtime rtr::editor)
endif()

add_executable(shadertoy runtime/shadertoy.cpp)
target_link_libraries(shadertoy PRIVATE rtr::runtime)

add_executable(gravity_reset runtime/gravity_reset.cpp)
target_link_libraries(gravity_reset PRIVATE rtr::runtime)

add_executable(torque_spin runtime/torque_spin.cpp)
target_link_libraries(torque_spin PRIVATE rtr::runtime)

add_executable(sphere_sphere_collision runtime/sphere_sphere_collision.cpp)
target_link_libraries(sphere_sphere_collision PRIVATE rtr::runtime)
