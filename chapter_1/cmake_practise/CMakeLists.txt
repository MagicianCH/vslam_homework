cmake_minimum_required(VERSION 3.1.0)
project(cmake_practise)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

include_directories(
    include
)

add_library(hello SHARED
    src/hello.cpp
)

add_executable(sayhello src/useHello.cpp)
target_link_libraries(sayhello hello)

install(TARGETS hello sayhello
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
)
install(FILES include/hello.h
    DESTINATION include
)