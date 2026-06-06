install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/numerixConfig.cmake
        DESTINATION ./numerix/cmake )
install(FILES ./VERSION
        DESTINATION ./numerix/ )
install(DIRECTORY ./include/
        DESTINATION ./numerix/include/)
install(DIRECTORY ${CMAKE_BINARY_DIR}/lib
        DESTINATION ./numerix)

install(FILES ./README.md
        DESTINATION ./)
install(FILES ./LICENSE
        DESTINATION ./)
