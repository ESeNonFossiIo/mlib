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

ADD_CUSTOM_TARGET(setup_astyle
  COMMAND sh ./scripts/setup_astyle.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(indent
  COMMAND sh ./scripts/indent
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(tag
  COMMAND ./scripts/tag
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(generate_readme
  COMMAND ./scripts/generate_readme
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
)

ADD_CUSTOM_TARGET(check_indentation
  COMMAND sh ./scripts/check_indentation.sh
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
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
    COMMAND ctest -R
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  )

ADD_CUSTOM_TARGET(run_python_tests
    COMMAND conda activate ${CMAKE_BINARY_DIR}/mlibpy_env
    COMMAND pytest ../python/tests
    COMMAND conda deactivate
    WORKING_DIRECTORY ${CMAKE_PROJECT_DIR}
  )
