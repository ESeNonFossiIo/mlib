
################################################################################
# -> RUN CHECK
################################################################################
EXECUTE_PROCESS(
  COMMAND ${TEST_PROG}
  OUTPUT_VARIABLE OUTFILE
  RESULT_VARIABLE HAD_ERROR
)

FILE ( WRITE output ${OUTFILE} )

IF(HAD_ERROR)
  FILE ( WRITE error ${HAD_ERROR} )
  MESSAGE(FATAL_ERROR " [ Test failed - no run ] ")
ELSE()
  MESSAGE( " -> Test compiled" )
ENDIF()

################################################################################
# -> DIFF CHECK
################################################################################
EXECUTE_PROCESS(
  COMMAND sh ${CMAKE_CURRENT_LIST_DIR}/../scripts/script_diff.sh
              ${TEST_NAME}.output
              ${TEST_DIR}/output
  OUTPUT_VARIABLE OUTFILE_DIFFERENT
  RESULT_VARIABLE DIFFERENT
)

IF(DIFFERENT)
  EXECUTE_PROCESS(
    COMMAND sh ${CMAKE_CURRENT_LIST_DIR}/../scripts/show_diff.sh
                ${TEST_NAME}.output
                ${TEST_DIR}/output
    OUTPUT_VARIABLE OUTFILE_DIFFERENT
    RESULT_VARIABLE DIFFERENT
  )
  MESSAGE(STATUS ${OUTFILE_DIFFERENT})
  MESSAGE(FATAL_ERROR " [ Test failed - files differ ] ")
ENDIF()
