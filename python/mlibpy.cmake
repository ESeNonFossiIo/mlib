FILE(COPY "./python/mlibpy" DESTINATION "${CMAKE_BINARY_DIR}")
# Make sure Python is installed
# find_package(Python REQUIRED)

# Activate conda environment, assume Anaconda or Miniconda is already installed
IF(EXISTS ${CMAKE_BINARY_DIR}/mlibpy_env)
    EXECUTE_PROCESS(COMMAND conda env update -f ${CMAKE_BINARY_DIR}/../python/mlibpy_env.yml --prefix ${CMAKE_BINARY_DIR}/mlibpy_env)
    EXECUTE_PROCESS(COMMAND conda activate ${CMAKE_BINARY_DIR}/mlibpy_env)
ELSE()
    EXECUTE_PROCESS(COMMAND conda env create -f ${CMAKE_BINARY_DIR}/../python/mlibpy_env.yml --prefix ${CMAKE_BINARY_DIR}/mlibpy_env)
    EXECUTE_PROCESS(COMMAND conda activate ${CMAKE_BINARY_DIR}/mlibpy_env)
ENDIF()
