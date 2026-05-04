INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/test_macro.cmake)

################################################################################
# -> ADD all test in this folder
################################################################################
FILE(GLOB SRC "*.cc")
FOREACH( FILE ${SRC})
  NEW_TEST( ${FILE} )
ENDFOREACH()
