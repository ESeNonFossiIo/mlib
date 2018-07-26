# Definitions:
################################################################################
IF (NOT DEFINED VAR_MLIB_ZERO_TOLERANCE)
    SET(VAR_MLIB_ZERO_TOLERANCE 1e-10)
ENDIF()
ADD_DEFINITIONS(-DVAR_MLIB_ZERO_TOLERANCE=${VAR_MLIB_ZERO_TOLERANCE})
