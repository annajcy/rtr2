find_package(Doxygen COMPONENTS dot)

if(DOXYGEN_FOUND)
  set(DOXYFILE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile")
  message(STATUS "Doxygen found, configured to generate RTR2 documentation.")
  add_custom_target(rtr_docs
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_SOURCE_DIR}/docs/api/doxygen"
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYFILE_PATH}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating RTR2 API documentation with Doxygen..."
    VERBATIM USES_TERMINAL
  )
else()
  message(WARNING "Doxygen not found. RTR2 documentation target will not be created. Try installing doxygen and graphviz.")
endif()
