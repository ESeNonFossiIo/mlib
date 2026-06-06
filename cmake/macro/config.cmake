# Config:
################################################################################
set(INCLUDE_INSTALL_DIR     include/    )
set(LIB_INSTALL_DIR         lib/        )
set(SYSCONFIG_INSTALL_DIR   etc/numerix/ )
include(CMakePackageConfigHelpers)
configure_package_config_file(./cmake/numerixConfig.cmake.in   ${CMAKE_CURRENT_BINARY_DIR}/cmake/numerixConfig.cmake
                              INSTALL_DESTINATION             ${LIB_INSTALL_DIR}/numerix/cmake
                              PATH_VARS INCLUDE_INSTALL_DIR SYSCONFIG_INSTALL_DIR)
# write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/cmake/numerixConfig${_version}.cmake
#                                  VERSION ${_version}
#                                  COMPATIBILITY SameMajorVersion )
