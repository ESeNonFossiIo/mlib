FIND_PACKAGE(Eigen3 REQUIRED)

# Modern Eigen3 (>= 3.3.7 installed as a CMake package) ships an imported
# target Eigen3::Eigen and does not always populate the legacy
# EIGEN3_INCLUDE_DIR variable.  Pull the include directories out of the
# target so both old and new layouts work.
IF(TARGET Eigen3::Eigen)
    GET_TARGET_PROPERTY(_eigen3_inc Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
    IF(_eigen3_inc)
        INCLUDE_DIRECTORIES(${_eigen3_inc})
        # `numerix/eigen/conversion.h` uses `#include <eigen3/Eigen/Dense>`, so
        # the parent of the eigen3/ subdir must also be on the include path.
        FOREACH(_inc ${_eigen3_inc})
            GET_FILENAME_COMPONENT(_inc_parent ${_inc} DIRECTORY)
            INCLUDE_DIRECTORIES(${_inc_parent})
        ENDFOREACH()
    ENDIF()
ENDIF()

IF(EIGEN3_INCLUDE_DIR)
    INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
ENDIF()
