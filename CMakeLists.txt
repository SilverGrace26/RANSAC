cmake_minimum_required(VERSION 3.10)
project(RansacProject)

find_package(Eigen3 REQUIRED)

add_executable(RL RANSAC_line.cpp)
add_executable(RP RANSAC_plane.cpp)

# target_link_libraries(RL Eigen3::Eigen)
target_link_libraries(RP Eigen3::Eigen)


