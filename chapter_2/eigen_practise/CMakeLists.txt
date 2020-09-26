cmake_minimum_required(VERSION 3.10)
project(eigen_practise)

set(CMAKE_BUILD_TYPE "Release")

find_package(Eigen3 REQUIRED)

include_directories(
    ${Eigen3_INCLUDE_DIRS}
)

add_executable(practise2 src/practise2.cpp)
target_link_libraries(practise2
    ${Eigen3_LIBS}
)

add_executable(practise3 src/practise3.cpp)
target_link_libraries(practise3
    ${Eigen3_LIBS}
)
