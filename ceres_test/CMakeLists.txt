cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ndt_matching)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ndt_matching NDT.cc)

target_link_libraries(ndt_matching ${PCL_LIBRARIES})
