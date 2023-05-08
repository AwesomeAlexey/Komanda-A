cmake_minimum_required(VERSION 3.2)

project(control_system CXX)
add_executable(control_system
    src/control_system.cpp
    src/sliding_controller.cpp
    inc/control_system/sliding_controller.h
)
target_link_libraries(control_system PUBLIC butterfly_robot utils)
target_include_directories(control_system PRIVATE inc)

add_executable(servo_request_state src/servo_request_state.cpp)
target_link_libraries(servo_request_state butterfly_robot)
target_include_directories(servo_request_state PRIVATE inc)

add_executable(servo_set_home_position src/servo_set_home_position.cpp)
target_link_libraries(servo_set_home_position butterfly_robot)
target_include_directories(servo_set_home_position PRIVATE inc)

add_executable(camera_request_state src/camera_request_state.cpp)
target_link_libraries(camera_request_state butterfly_robot)
target_include_directories(camera_request_state PRIVATE inc)

add_subdirectory(tests)
