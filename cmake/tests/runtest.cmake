################################################################################
# -> RUN CHECK
################################################################################
EXECUTE_PROCESS(
  COMMAND ${TEST_PROG}
  OUTPUT_VARIABLE OUTFILE
  RESULT_VARIABLE HAD_ERROR
)

FILE ( WRITE ${TEST_DIR}/output ${OUTFILE} )

IF(HAD_ERROR)
  FILE ( WRITE ${TEST_DIR}/error ${HAD_ERROR} )
  MESSAGE(FATAL_ERROR " [ Test failed - no run ] ")
ELSE()
  MESSAGE( " -> Test compiled" )
ENDIF()

################################################################################
# -> DIFF CHECK
################################################################################
# 1. Search the host OS for diffing tools
find_program(NUMDIFF_CMD numdiff)
find_program(DIFF_CMD diff)

set(EXPECTED_FILE "${TEST_NAME}.output")
set(ACTUAL_FILE "${TEST_DIR}/output")

# 2. Replicate the script logic directly in CMake
if(NUMDIFF_CMD)
    # Use numdiff with tolerance if installed
    set(TOLERANCE "1.0e-6")
    execute_process(
        COMMAND ${NUMDIFF_CMD} -V -r ${TOLERANCE} ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
elseif(DIFF_CMD)
    # Fallback to standard diff, but ignore Windows CR line endings
    execute_process(
        COMMAND ${DIFF_CMD} --strip-trailing-cr ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
else()
    # Ultimate fallback: Pure CMake text diff (ignores line endings)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
    )
    if(DIFFERENT)
        set(OUTFILE_DIFFERENT "Files differ natively. Install 'numdiff' or 'diff' for a detailed report.")
    endif()
endif()

# 3. Print the diff report if it failed
IF(DIFFERENT)
  # Mimic the output formatting from show_diff.sh
  MESSAGE(STATUS "===================================INIT==================================")
  MESSAGE(STATUS "${OUTFILE_DIFFERENT}")
  if(ERROR_DIFFERENT)
      MESSAGE(STATUS "${ERROR_DIFFERENT}")
  endif()
  MESSAGE(STATUS "===================================END===================================")
  
  # Write the .diff file mimic-ing the behavior of check_diff
  FILE(WRITE "${ACTUAL_FILE}.diff" "${OUTFILE_DIFFERENT}\n${ERROR_DIFFERENT}")
  
  MESSAGE(FATAL_ERROR " [ Test failed - files differ ] ")
ENDIF()