if (TARGET rtr_editor)
  add_executable(framework_quickstart_main framework_quickstart_main.cpp)
  target_link_libraries(framework_quickstart_main PRIVATE rtr::runtime rtr::editor)

  add_executable(shadertoy_main shadertoy_main.cpp)
  target_link_libraries(shadertoy_main PRIVATE rtr::runtime rtr::editor)
endif()

add_executable(pbpt_cbox_roundtrip_main pbpt_cbox_roundtrip_main.cpp)
target_link_libraries(pbpt_cbox_roundtrip_main PRIVATE rtr::runtime)

if (TARGET rtr_editor)
  add_executable(framework_offline_cbox_main framework_offline_cbox_main.cpp)
  target_link_libraries(framework_offline_cbox_main PRIVATE rtr::runtime rtr::editor)
endif()
