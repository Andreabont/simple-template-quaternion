cmake_minimum_required(VERSION 3.0)
project(quaternion)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(Boost COMPONENTS unit_test_framework system REQUIRED)
include_directories (${Boost_INCLUDE_DIRS})

add_executable(quaternion_example example.cpp Quaternion.h)

add_executable(quaternion_test test.cpp Quaternion.h)

target_link_libraries(quaternion_test ${Boost_LIBRARIES})

add_test(NAME quaternion_test WORKING_DIRECTORY ${PROJECT_BINARY_DIR} COMMAND ${PROJECT_BINARY_DIR}/quaternion_test)

enable_testing()
