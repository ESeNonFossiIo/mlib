# Require C++11:
################################################################################
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    SET(CMAKE_CXX_STANDARD 11)
    if(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 /W4")
      ADD_DEFINITIONS(/D_CRT_SECURE_NO_WARNINGS)
    else()
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Werror -Wall -Wextra -Wpedantic -Wdeprecated -pedantic-errors")
    endif()
    ADD_DEFINITIONS(-DWITH_CPP11)
endif()
