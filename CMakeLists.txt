
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
set(CMAKE_MACOSX_RPATH 1)

IF (WIN32)
  ADD_DEFINITIONS(-DWINDOWS)
  SET(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
  SET(BUILD_SHARED_LIBS TRUE)
ELSE()
  # set stuff for other systems
ENDIF()

INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/utility.cmake)

################################################################################
################################################################################
SET(_project "mlib")
FILE (READ "VERSION" _version)
STRING(REPLACE "\n" "\n      " _version ${_version})
PROJECT(${_project} CXX)

MESSAGE( "  ")
MESSAGE(STATUS "###############################################################")
MESSAGE( "  ")
MESSAGE(STATUS "  ${_project} : \n      ${_version}")
MESSAGE(STATUS "###############################################################")
MESSAGE( "  ")
################################################################################
################################################################################

# CXX flags
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/cxx_flags.cmake)

# DEFINITIONS
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/definitions.cmake)

# DEBUG FLAGS
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/debug_flags.cmake)

# Qt creator
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/qt_creator.cmake)
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/qt_support.cmake)

# Include files:
################################################################################
INCLUDE_DIRECTORIES("include")

aux_source_directory(. SRC_LIST)
FILE(GLOB_RECURSE _h_files   "include/*.h")
FILE(GLOB_RECURSE _cc_files  "source/*.cc")

SET(_project_lib "_${_project}")
ADD_LIBRARY(${_project_lib} SHARED ${_cc_files})
# target_link_libraries (${_project_lib})
set_target_properties(${_project_lib} PROPERTIES LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
set_target_properties(${_project_lib} PROPERTIES OUTPUT_NAME "${_project_lib}${BUILD_POSTFIX}")
target_qt_support(${_project_lib})

IF (WIN32)
  SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/lib CACHE
    PATH "${CMAKE_BINARY_DIR}/lib" FORCE)
  SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY
      ${CMAKE_BINARY_DIR}/lib CACHE
    PATH "${CMAKE_BINARY_DIR}/lib" FORCE)
ENDIF()
file(COPY "./include" DESTINATION "${CMAKE_BINARY_DIR}")

# External libraries files:
################################################################################
SET(external_libraries "")
SET(external_libraries_names "")

# VTK
OPTION( WITH_VTK
        "Use VtK Libraries" OFF)
IF(WITH_VTK)
    SET(external_libraries_names ${external_libraries_names} "VTK")
endif(WITH_VTK)

SET(PCL_WITH_VTK FALSE)
# PCL
OPTION( WITH_PCL
        "Use PCL Libraries" OFF)
IF(WITH_PCL)
    IF(WITH_VTK)
      SET(PCL_WITH_VTK TRUE)
      ADD_DEFINITIONS(-D_MYLIB_USE_PCL_WITH_VTK=ON)
    ENDIF(WITH_VTK)
    ADD_DEFINITIONS(-D_MYLIB_USE_PCL=ON)
    INCLUDE(${CMAKE_SOURCE_DIR}/cmake/libraries/pcl.cmake)
    IF(PCL_WITH_VTK)
      SET(external_libraries_names ${external_libraries_names} "PCL")
      SET(external_libraries_names ${external_libraries_names} "  - WITH VTK")
    ELSE(PCL_WITH_VTK)
      SET(external_libraries_names ${external_libraries_names} "PCL")
      SET(external_libraries_names ${external_libraries_names} "  - WITHOUT VTK")
    ENDIF(PCL_WITH_VTK)
    SET(external_libraries ${external_libraries} ${PCL_COMMON_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${PCL_SEGMENTATION_LIBRARIES} ${PCL_IO_LIBRARIES})
endif(WITH_PCL)

# Eigen3
OPTION( WITH_EIGEN3
        "Use Eigen3 Libraries" OFF)
IF(WITH_EIGEN3)
    ADD_DEFINITIONS(-D_MYLIB_USE_EIGEN3=ON)
    SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/libraries/find/FindEigen3.cmake)
    INCLUDE(${CMAKE_SOURCE_DIR}/cmake/libraries/eigen.cmake)
    SET(external_libraries_names ${external_libraries_names} "EIGEN3")
    SET(external_libraries ${external_libraries} ${EIGEN3_LIBRARIES})
endif(WITH_EIGEN3)

# Main files:
################################################################################
FILE(GLOB_RECURSE _main "main/*.cc")

OPTION( ENABLE_ALL_MAIN
        "Enable all executable" OFF)

FOREACH(_m ${_main})
  GET_FILENAME_COMPONENT(_main_name ${_m} NAME_WE)
  IF(${_main_name} STREQUAL "main")
    OPTION( ENABLE_MAIN_${_main_name}
            "Enable ${_main_name}" ON)
  ELSE()
    OPTION( ENABLE_MAIN_${_main_name}
            "Enable ${_main_name}" OFF)
  ENDIF()
  IF(ENABLE_ALL_MAIN)
    SET( ENABLE_MAIN_${_main_name} ON
         CACHE BOOL "-- Executable ${_main_name} enabled" FORCE)
  ENDIF()
ENDFOREACH()

TARGET_LINK_LIBRARIES(${_project_lib} ${external_libraries})
FOREACH(_m ${_main})
  GET_FILENAME_COMPONENT(_main_name ${_m} NAME_WE)
  IF(ENABLE_MAIN_${_main_name})
    ADD_EXECUTABLE(${_main_name} ${_m})
    TARGET_LINK_LIBRARIES(${_main_name} ${_project_lib})
  ENDIF()
ENDFOREACH()

DIRLIST(_gui_dirs "${CMAKE_SOURCE_DIR}/gui")

OPTION( ENABLE_ALL_Qt_MAIN
        "Enable Qt gui app" OFF)
FOREACH(_main_name ${_gui_dirs})
  IF(${_main_name} STREQUAL "main")
    OPTION( ENABLE_MAIN_${_main_name}
            "Enable ${_main_name}" ON)
  ELSE()
    OPTION( ENABLE_MAIN_${_main_name}
            "Enable ${_main_name}" OFF)
  ENDIF()
  IF(ENABLE_ALL_MAIN)
    SET( ENABLE_MAIN_${_main_name} ON
         CACHE BOOL "-- Executable ${_main_name} enabled" FORCE)
  ENDIF()
ENDFOREACH()

FOREACH(_main_name ${_gui_dirs})
  IF(ENABLE_MAIN_${_main_name})
    ADD_SUBDIRECTORY(${CMAKE_SOURCE_DIR}/gui/${_main_name}/)
  ENDIF()
ENDFOREACH()

# Targets
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/targets.cmake)

# Tests
INCLUDE(tests/tests.cmake)

# Documentation
INCLUDE(doxygen/doxygen.cmake)

# Status
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/status.cmake)

# Config
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/config.cmake)

# Install
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/install.cmake)

# packaging
INCLUDE(${CMAKE_SOURCE_DIR}/cmake/macro/cpack.cmake)

MESSAGE(STATUS "Libraries used:")
FOREACH(lib ${external_libraries_names})
  MESSAGE(STATUS " > ${lib}")
ENDFOREACH()
