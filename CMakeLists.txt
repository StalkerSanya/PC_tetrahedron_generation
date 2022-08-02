cmake_minimum_required(VERSION 3.13)
project(PC_tetrahedron_generation)

set(CMAKE_CXX_STANDARD 14)
find_package(PCL 1.2 REQUIRED)

add_executable (PC_gen main.cpp)
target_link_libraries (PC_gen ${PCL_LIBRARIES})
