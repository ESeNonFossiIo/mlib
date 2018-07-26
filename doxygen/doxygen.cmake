# Documentation:
################################################################################
# add a target to generate API documentation with Doxygen
FIND_PACKAGE(Doxygen)
IF(DOXYGEN_FOUND)
  MESSAGE(STATUS " Doxygen FOUND")
  FILE (READ "VERSION" CURRENTMLIB_VERSION)
  STRING(REPLACE "\n" "\n      " CURRENTMLIB_VERSION ${CURRENTMLIB_VERSION})
  STRING(REPLACE "Version ............ " "" CURRENTMLIB_VERSION ${CURRENTMLIB_VERSION})
  CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/doxygen/Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
  ADD_CUSTOM_TARGET(doc
    ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMENT "Generating API documentation with Doxygen" VERBATIM
    # ENV MYLIB_VERSION=${MYLIB_VERSION}
  )
ENDIF(DOXYGEN_FOUND)
