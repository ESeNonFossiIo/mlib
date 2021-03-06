# Status:
################################################################################
MESSAGE( " " )
MESSAGE(STATUS "========================== STATUS: ==========================")
MESSAGE(STATUS " CMAKE_CXX_FLAGS : ")
string(REPLACE " " ";" CMAKE_CXX_FLAGS_LIST ${CMAKE_CXX_FLAGS})
FOREACH(_m ${CMAKE_CXX_FLAGS_LIST})
  MESSAGE(STATUS "   \t\t-> " ${_m})
ENDFOREACH()
MESSAGE(STATUS " MAIN : ")
FOREACH(_m ${_main})
  GET_FILENAME_COMPONENT(_main_name ${_m} REALPATH)
  STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}/main/" "" _main_name ${_main_name} )
  STRING(REGEX REPLACE ".cc" "" _main_name ${_main_name} )

  IF(ENABLE_MAIN_${_main_name})
    MESSAGE(STATUS "   \t\t[x] " ${_main_name})
  ELSE()
    MESSAGE(STATUS "   \t\t[ ] " ${_main_name})
  ENDIF()
ENDFOREACH()
FOREACH(_main_name ${_gui_dirs})
  IF(ENABLE_MAIN_${_main_name})
    MESSAGE(STATUS "   \t\t[x] " ${_main_name})
  ELSE()
    MESSAGE(STATUS "   \t\t[ ] " ${_main_name})
  ENDIF()
ENDFOREACH()
MESSAGE(STATUS " TESTS : ")
FOREACH(_m ${_test_dirs})
  GET_FILENAME_COMPONENT(_test_dirs_name ${_m} REALPATH)
  STRING(REGEX REPLACE "${CMAKE_SOURCE_DIR}/" "" _test_dirs_name ${_test_dirs_name} )

IF(ENABLE_TESTS_${_test_dirs_name})
    MESSAGE(STATUS "   \t\t[x] " ${_test_dirs_name})
  ELSE()
    MESSAGE(STATUS "   \t\t[ ] " ${_test_dirs_name})
  ENDIF()
ENDFOREACH()
MESSAGE(STATUS "=============================================================")
MESSAGE( " " )
