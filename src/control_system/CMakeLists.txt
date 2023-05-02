cmake_minimum_required(VERSION 3.2)

project(control_system CXX)
add_executable(sliding_controller
    src/control_system.cpp
    inc/control_system/sliding_controller.h
)
target_link_libraries(sliding_controller
    butterfly_robot
)
target_include_directories(sliding_controller PRIVATE inc)
add_subdirectory(tests)
