FILE(COPY "./python/mlibpy" DESTINATION "${CMAKE_BINARY_DIR}")
# Make sure Python is installed
# find_package(Python REQUIRED)

# Activate conda environment, assume Anaconda or Miniconda is already installed
IF(EXISTS ${CMAKE_BINARY_DIR}/mlibpy_env)
    EXECUTE_PROCESS(COMMAND conda activate ${CMAKE_BINARY_DIR}/mlibpy_env)
ELSE()
    EXECUTE_PROCESS(COMMAND conda create --yes --quiet --prefix ${CMAKE_BINARY_DIR}/mlibpy_env  python=3.6)
    EXECUTE_PROCESS(COMMAND conda activate ${CMAKE_BINARY_DIR}/mlibpy_env)
ENDIF()
