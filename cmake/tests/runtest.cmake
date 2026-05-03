################################################################################
# -> DIFF CHECK (Best of both worlds: numdiff -> Python fallback)
################################################################################
# 1. Search the host OS for diffing tools
find_program(NUMDIFF_CMD numdiff)
find_program(NDIFF_CMD ndiff)
find_package(Python3 COMPONENTS Interpreter QUIET) # QUIET means it won't crash if Python is missing

set(EXPECTED_FILE "${TEST_NAME}.output")
set(ACTUAL_FILE "${TEST_DIR}/output")
set(TOLERANCE "1.0e-6")

# 2. Execute the best available tool
if(NUMDIFF_CMD)
    # Priority 1: numdiff (Highly precise, native speed)
    execute_process(
        COMMAND ${NUMDIFF_CMD} -V -r ${TOLERANCE} ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
elseif(NDIFF_CMD)
    # Priority 2: ndiff 
    execute_process(
        COMMAND ${NDIFF_CMD} --relative-error ${TOLERANCE} ${EXPECTED_FILE} ${ACTUAL_FILE}
        RESULT_VARIABLE DIFFERENT
        OUTPUT_VARIABLE OUTFILE_DIFFERENT
        ERROR_VARIABLE ERROR_DIFFERENT
    )
elseif(Python3_FOUND)
    # Priority 3: Python fallback (Cross-platform, handles Windows math perfectly)
    set(PYTHON_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/../scripts/script_diff.py")
    execute_process(
        COMMAND ${Python3_EXECUTABLE} ${PYTHON_SCRIPT} ${EXPECTED_FILE} ${ACTUAL_FILE} ${TOLERANCE}
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
