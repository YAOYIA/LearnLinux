cmake_minimum_required(VERSION 3.10)
project(test_project)

set(CMAKE_CXX_STANDARD 14)

find_package(GTest REQUIRED)
include_directories(${GTEST_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/include)
add_executable(test src/my_add.cc test/test.cc)

target_link_libraries(test GTest::GTest GTest::Main pthread)