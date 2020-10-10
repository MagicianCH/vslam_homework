cmake_minimum_required(VERSION 2.8)
project(draw_trajectory)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

add_compile_options(--std=c++11)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(
    ${Pangolin_INCLUDE_DIRS}
    ${Sophus_INCLUDE_DIRS}
)

add_executable(draw_trajectory
    src/draw_trajectory.cpp
)
target_link_libraries(draw_trajectory
    ${Sophus_LIBRARIES}
    ${Pangolin_LIBRARIES}
)

add_executable(trajectory_error
    src/trajectory_error.cpp
)
target_link_libraries(trajectory_error
    ${Sophus_LIBRARIES}
)
