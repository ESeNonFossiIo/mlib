# ==============================================================================
# runtest.cmake
# Cross-platform test execution and diffing script.
# Expected variables passed from CMake macro:
#   TEST_PROG : Absolute path to the test executable
#   TEST_DIR  : Absolute path to the isolated working directory for this test
#   TEST_NAME : Base path/name for the expected .output file
# ==============================================================================

################################################################################
# -> RUN CHECK (Robust Error Handling)
################################################################################
# Execute the test program and capture all output streams
EXECUTE_PROCESS(
  COMMAND ${TEST_PROG}
  OUTPUT_VARIABLE OUTFILE
  ERROR_VARIABLE ERRFILE    
  RESULT_VARIABLE HAD_ERROR
)

# ALWAYS safely create the output file, even if the program crashed and output is empty.
# Using quotes ensures empty strings don't cause CMake syntax errors.
FILE ( WRITE "${TEST_DIR}/output" "${OUTFILE}" )

# Strict string comparison to catch crashes (segfaults, missing DLLs, etc.)
IF(NOT "${HAD_ERROR}" STREQUAL "0")
  # Write the error log to disk for CI artifact inspection
  FILE ( WRITE "${TEST_DIR}/error" "Exit Code: ${HAD_ERROR}\nStderr: ${ERRFILE}" )
  MESSAGE(FATAL_ERROR " [ Test failed - execution error ] \nExit Code: ${HAD_ERROR}\nError Log:\n${ERRFILE}")
ELSE()
  MESSAGE( " -> Test ran successfully" )
ENDIF()


################################################################################
# -> DIFF CHECK (Cascading: numdiff -> ndiff -> Python -> Native CMake)
################################################################################
# 1. Search the host OS for mathematical diffing tools
find_program(NUMDIFF_CMD numdiff)
find_program(NDIFF_CMD ndiff)
find_package(Python3 COMPONENTS Interpreter QUIET)

set(EXPECTED_FILE "${TEST_NAME}.output")
set(ACTUAL_FILE "${TEST_DIR}/output")
set(TOLERANCE "1.0e-6")

# 2. Execute the most precise tool available on this machine
if(NUMDIFF_CMD)
    # Priority 1: numdiff (Highly precise math comparison)
    execute_process(
        COMMAND ${NUMDIFF_CMD} -V -r ${TOLERANCE} ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
elseif(NDIFF_CMD)
    # Priority 2: ndiff (Alternative math comparison)
    execute_process(
        COMMAND ${NDIFF_CMD} --relative-error ${TOLERANCE} ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
elseif(Python3_FOUND)
    # Priority 3: Python script (Cross-platform fallback, handles Windows math perfectly)
    set(PYTHON_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../scripts/script_diff.py")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${PYTHON_SCRIPT} ${EXPECTED_FILE} ${ACTUAL_FILE} ${TOLERANCE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
else()
    # Ultimate fallback: Pure CMake text diff (ignores line endings, but strict on math)
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E compare_files --ignore-eol ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
    )
    if(DIFFERENT)
        set(OUTFILE_DIFFERENT "Files differ natively. Install 'numdiff', 'ndiff', or Python for a detailed math diff.")
    endif()
endif()

# 3. Print the diff report and fail the test if differences were found
IF(DIFFERENT)
  MESSAGE(STATUS "===================================INIT==================================")
  
  if(NUMDIFF_CMD OR NDIFF_CMD OR Python3_FOUND)
      MESSAGE(STATUS "${OUTFILE_DIFFERENT}")
      if(ERROR_DIFFERENT)
          MESSAGE(STATUS "${ERROR_DIFFERENT}")
      endif()
  else()
      MESSAGE(STATUS "${OUTFILE_DIFFERENT}")
  endif()
  
  MESSAGE(STATUS "===================================END===================================")
  
  # Write the .diff file to the disk for manual inspection later
  FILE(WRITE "${ACTUAL_FILE}.diff" "${OUTFILE_DIFFERENT}\n${ERROR_DIFFERENT}")
  
  MESSAGE(FATAL_ERROR " [ Test failed - files differ ] ")
ENDIF()