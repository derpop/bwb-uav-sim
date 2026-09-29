cmake_minimum_required(VERSION 3.16)
project(bwb_uav_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)
endif()

add_library(bwb_sim sim/src/rigid_body.cpp)
target_include_directories(bwb_sim PUBLIC sim/include)

add_executable(tumble sim/apps/tumble.cpp)
target_link_libraries(tumble PRIVATE bwb_sim)

enable_testing()
add_executable(test_rigid_body sim/tests/test_rigid_body.cpp)
target_link_libraries(test_rigid_body PRIVATE bwb_sim)
add_test(NAME rigid_body COMMAND test_rigid_body)
