cmake_minimum_required(VERSION 3.16)

project(bwb_uav_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(test_rigid_body sim/test/test_rigid_body.cpp)

add_test(NAME rigid_body COMMAND test_rigid_body)

