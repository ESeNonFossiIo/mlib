# Targets:
################################################################################
FOREACH(_m ${_main})
  GET_FILENAME_COMPONENT(_main_name ${_m} NAME_WE)
  ADD_CUSTOM_TARGET(run_${_main_name}
    COMMAND ${_main_name}
    DEPENDS ${_main_name}
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  )
ENDFOREACH()

ADD_CUSTOM_TARGET(clang_format
  COMMAND bash ./scripts/clang_format.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Reformat all C/C++ sources with clang-format"
)

ADD_CUSTOM_TARGET(tag
  COMMAND ./scripts/tag
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(generate_readme
  COMMAND ./scripts/generate_readme
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(check_format
  COMMAND bash ./scripts/clang_format.sh --check
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  COMMENT "Check formatting with clang-format (no files modified)"
)

ADD_CUSTOM_TARGET(debug
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Debug ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Debug"
  )

ADD_CUSTOM_TARGET(release
  COMMAND ${CMAKE_COMMAND} -DCMAKE_BUILD_TYPE=Release ${CMAKE_SOURCE_DIR}
  COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target all
  COMMENT "Switch CMAKE_BUILD_TYPE to Release"
  )

ADD_CUSTOM_TARGET(run_tests
    COMMAND ${CMAKE_MAKE_PROGRAM}
    COMMAND ctest -R ".*" --output-on-failure
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  )

ADD_CUSTOM_TARGET(run_python_tests
    COMMAND pytest ../python/tests
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  )
