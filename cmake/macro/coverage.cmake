# Coverage flags
################################################################################
# Enable with: cmake -D WITH_COVERAGE=ON ...
#
# Adds --coverage to compile and link flags (works with both gcc/gcov and
# clang/llvm-cov gcov). A `coverage` custom target runs gcovr against the
# library sources and writes an HTML report under <build>/coverage/html.
#
# Tools required at run time (build is unaffected):
#   * gcovr        — driver, reads .gcno/.gcda
#   * gcov OR llvm-cov gcov — toolchain matched to compiler
################################################################################
OPTION(WITH_COVERAGE "Build with coverage instrumentation" OFF)

IF(WITH_COVERAGE)
  IF(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    MESSAGE(WARNING
      "WITH_COVERAGE is ON but CMAKE_BUILD_TYPE is '${CMAKE_BUILD_TYPE}'. "
      "Coverage numbers are most accurate with -O0 (Debug).")
  ENDIF()

  IF(MSVC)
    MESSAGE(FATAL_ERROR "WITH_COVERAGE is not supported with MSVC.")
  ENDIF()

  # Coverage must be measured at -O0: with optimisation the compiler inlines
  # trivial constructors/destructors and rewrites conditionals, which makes
  # gcov mis-attribute function and branch hits. The build-type flags
  # (e.g. CMAKE_CXX_FLAGS_RELEASE = "-O3 -DNDEBUG") are appended *after*
  # CMAKE_CXX_FLAGS, so an -O0 there would be overridden. Strip the
  # optimisation level from every per-configuration flag set first; -DNDEBUG
  # is kept so assert() behaviour matches the chosen build type.
  FOREACH(_cfg "" "_DEBUG" "_RELEASE" "_RELWITHDEBINFO" "_MINSIZEREL")
    FOREACH(_lang CXX C)
      STRING(REGEX REPLACE "-O[0-9sgz]" "" CMAKE_${_lang}_FLAGS${_cfg}
             "${CMAKE_${_lang}_FLAGS${_cfg}}")
    ENDFOREACH()
  ENDFOREACH()

  # --coverage implies -fprofile-arcs -ftest-coverage and links libgcov.
  # -fprofile-update=atomic avoids counter races when tests fork or thread.
  SET(_cov_flags "--coverage -O0 -g -fprofile-update=atomic")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${_cov_flags}")
  SET(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS} ${_cov_flags}")
  SET(CMAKE_EXE_LINKER_FLAGS    "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")

  # Pick a gcov front-end that matches the compiler.
  IF(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    FIND_PROGRAM(LLVM_COV_PROG llvm-cov)
    IF(LLVM_COV_PROG)
      SET(_gcov_executable "${LLVM_COV_PROG} gcov")
    ELSE()
      MESSAGE(WARNING
        "Compiler is clang but llvm-cov was not found; falling back to gcov "
        "which will likely fail with a version mismatch.")
      SET(_gcov_executable "gcov")
    ENDIF()
  ELSE()
    SET(_gcov_executable "gcov")
  ENDIF()

  FIND_PROGRAM(GCOVR_PROG gcovr)
  IF(GCOVR_PROG)
    SET(_cov_html_dir "${CMAKE_BINARY_DIR}/coverage/html")
    SET(_cov_xml      "${CMAKE_BINARY_DIR}/coverage/coverage.xml")
    SET(_cov_txt      "${CMAKE_BINARY_DIR}/coverage/coverage.txt")

    ADD_CUSTOM_TARGET(coverage
      COMMAND ${CMAKE_COMMAND} -E make_directory ${_cov_html_dir}
      COMMAND ${GCOVR_PROG}
              --root ${CMAKE_SOURCE_DIR}
              --gcov-executable "${_gcov_executable}"
              --filter "${CMAKE_SOURCE_DIR}/source/"
              --filter "${CMAKE_SOURCE_DIR}/include/mlib/"
              --exclude ".*/tests/.*"
              --print-summary
              --txt        ${_cov_txt}
              --html-details ${_cov_html_dir}/index.html
              --xml        ${_cov_xml}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Generating coverage report in ${_cov_html_dir}"
      VERBATIM)
  ELSE()
    MESSAGE(STATUS "gcovr not found: 'coverage' target will not be available.")
  ENDIF()

  MESSAGE(STATUS "Coverage instrumentation enabled (--coverage).")
ENDIF()
