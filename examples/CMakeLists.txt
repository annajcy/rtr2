function(rtr_configure_example TARGET_NAME)
  target_link_libraries(${TARGET_NAME} PRIVATE ${ARGN})
  if(TARGET compile_slang_shaders)
    add_dependencies(${TARGET_NAME} compile_slang_shaders)
  endif()
endfunction()

if (TARGET rtr_editor)
  add_executable(quickstart editor/quickstart.cpp)
  rtr_configure_example(quickstart rtr::runtime rtr::editor)

  add_executable(offline_cbox editor/offline_cbox.cpp)
  rtr_configure_example(offline_cbox rtr::runtime rtr::editor)

  add_executable(shadertoy_editor editor/shadertoy_editor.cpp)
  rtr_configure_example(shadertoy_editor rtr::runtime rtr::editor)

  add_executable(editable_shadertoy_editor editor/editable_shadertoy_editor.cpp)
  rtr_configure_example(editable_shadertoy_editor rtr::runtime rtr::editor)

  add_executable(gravity_reset_editor editor/gravity_reset_editor.cpp)
  rtr_configure_example(gravity_reset_editor rtr::runtime rtr::editor)

  add_executable(gravity_spin_editor editor/gravity_spin_editor.cpp)
  rtr_configure_example(gravity_spin_editor rtr::runtime rtr::editor)

  add_executable(sphere_box_collision_editor editor/sphere_box_collision_editor.cpp)
  rtr_configure_example(sphere_box_collision_editor rtr::runtime rtr::editor)

  add_executable(sphere_pen_collision_editor editor/sphere_pen_collision_editor.cpp)
  rtr_configure_example(sphere_pen_collision_editor rtr::runtime rtr::editor)

  add_executable(plane_collision_editor editor/plane_collision_editor.cpp)
  rtr_configure_example(plane_collision_editor rtr::runtime rtr::editor)

  add_executable(physics_showcase_editor editor/physics_showcase_editor.cpp)
  rtr_configure_example(physics_showcase_editor rtr::runtime rtr::editor)

  add_executable(deformable_mesh_editor editor/deformable_mesh_editor.cpp)
  rtr_configure_example(deformable_mesh_editor rtr::runtime rtr::editor)

  add_executable(ipc_fixed_end_block_editor editor/ipc_fixed_end_block_editor.cpp)
  rtr_configure_example(ipc_fixed_end_block_editor rtr::runtime rtr::editor)

  if(RTR_BUILD_FTETWILD)
    add_executable(ipc_hanging_bunny_editor editor/ipc_hanging_bunny_editor.cpp)
    rtr_configure_example(ipc_hanging_bunny_editor rtr::runtime rtr::editor)
  endif()

  add_executable(lab1_angry_bunny games103_lab/lab1_angry_bunny/lab1_angry_bunny.cpp)
  rtr_configure_example(lab1_angry_bunny rtr::runtime rtr::editor)
endif()

add_executable(shadertoy runtime/shadertoy.cpp)
rtr_configure_example(shadertoy rtr::runtime)

add_executable(gravity_reset runtime/gravity_reset.cpp)
rtr_configure_example(gravity_reset rtr::runtime)

add_executable(torque_spin runtime/torque_spin.cpp)
rtr_configure_example(torque_spin rtr::runtime)

add_executable(sphere_sphere_collision runtime/sphere_sphere_collision.cpp)
rtr_configure_example(sphere_sphere_collision rtr::runtime)

add_executable(ipc_offline_render headless/ipc_offline_render.cpp)
rtr_configure_example(ipc_offline_render rtr::runtime)

add_executable(ipc_offline_preview headless/ipc_offline_preview.cpp)
rtr_configure_example(ipc_offline_preview rtr::runtime)
