cmake_minimum_required(VERSION 3.2)

project(control_system CXX)
add_executable(control_system
    src/control_system.cpp
    inc/control_system/sliding_controller.h
)
target_link_libraries(control_system
    butterfly_robot
)
target_include_directories(control_system PRIVATE inc)
add_subdirectory(tests)
