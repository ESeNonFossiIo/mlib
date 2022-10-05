# Require C++11:
################################################################################
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    SET(CMAKE_CXX_STANDARD 11)
    if(MSVC)
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 /W4 /WX")
    else()
      SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Wextra -Wpedantic -Wdeprecated -pedantic-errors")
    endif()
    add_definitions(-DWITH_CPP11)
endif()
