# Definitions:
################################################################################
IF (NOT DEFINED VAR_NUMERIX_ZERO_TOLERANCE)
    SET(VAR_NUMERIX_ZERO_TOLERANCE 1e-10)
ENDIF()
ADD_DEFINITIONS(-DVAR_NUMERIX_ZERO_TOLERANCE=${VAR_NUMERIX_ZERO_TOLERANCE})
