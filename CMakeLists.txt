cmake_minimum_required(VERSION 3.16)

project(bwb_uav_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wshadow)


enable_testing()

add_executable(test_rigid_body sim/test/test_rigid_body.cpp)
add_executable(tumble_demo sim/apps/tumble_demo.cpp)
add_executable(test_aero sim/test/test_aero.cpp)
add_library(bwb_sim sim/src/rigid_body.cpp sim/src/aero.cpp)
target_include_directories(bwb_sim PUBLIC sim/include)
target_link_libraries(test_rigid_body PRIVATE bwb_sim)
target_link_libraries(tumble_demo PRIVATE bwb_sim)
target_link_libraries(test_aero PRIVATE bwb_sim)

add_test(NAME rigid_body COMMAND test_rigid_body)
add_test(NAME aero COMMAND test_aero)
