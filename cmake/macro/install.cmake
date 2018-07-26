install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/mlibConfig.cmake
        DESTINATION ./mlib/cmake )
install(FILES ./VERSION
        DESTINATION ./mlib/ )
install(DIRECTORY ./include/
        DESTINATION ./mlib/include/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib
        DESTINATION ./mlib)

install(FILES ./README.md
        DESTINATION ./)
install(FILES ./LICENSE
        DESTINATION ./)
