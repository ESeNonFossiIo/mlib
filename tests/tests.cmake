# Tests:
################################################################################
SUBDIRLIST(_test_dirs "${CMAKE_SOURCE_DIR}/tests")

OPTION( ENABLE_ALL_TESTS
        "Enable all tests" OFF)

FOREACH(_m ${_test_dirs})
  GET_FILENAME_COMPONENT(_test_dirs_name ${_m} REALPATH)
  STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}/" "" _test_dirs_name ${_test_dirs_name} )

  IF(${_test_dirs_name} STREQUAL "main")
    OPTION( ENABLE_TESTS_${_test_dirs_name}
            "Enable ${_test_dirs_name}" ON)
  ELSE()
    OPTION( ENABLE_TESTS_${_test_dirs_name}
            "Enable ${_test_dirs_name}" OFF)
  ENDIF()
  IF(ENABLE_ALL_TESTS)
    SET( ENABLE_TESTS_${_test_dirs_name} ON
         CACHE BOOL "-- Executable ${_main_name} enabled" FORCE)
  ENDIF()

  FOREACH(_n ${_test_dirs})
    GET_FILENAME_COMPONENT(_test_dirs_name_n ${_n} REALPATH)
    STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}/" "" _test_dirs_name_n ${_test_dirs_name_n} )
    STRING(FIND "ENABLE_TESTS_${_test_dirs_name_n}" "ENABLE_TESTS_${_test_dirs_name}" pos)

    IF(${pos} STREQUAL "0" AND  ENABLE_TESTS_${_test_dirs_name})
      SET( ENABLE_TESTS_${_test_dirs_name_n} ON
           CACHE BOOL "-- Executable ${_main_name} enabled" FORCE)
    ENDIF()
  ENDFOREACH()
ENDFOREACH()

FOREACH(_m ${_test_dirs})
  GET_FILENAME_COMPONENT(_test_dirs_name ${_m} NAME_WE)
  IF(ENABLE_TESTS_${_m})
        ENABLE_TESTING()
        STRING(REGEX REPLACE "-" "/" _m_name ${_m} )
        ADD_SUBDIRECTORY(tests/${_m}/)
        STRING(REGEX REPLACE "/" "-" _m_name ${_m} )
        ADD_CUSTOM_TARGET(fix_test_${_m_name}
          COMMAND sh ./scripts/fix_tests ${_m} ${CMAKE_CURRENT_BINARY_DIR}
          WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
        )
  ENDIF()
ENDFOREACH()
