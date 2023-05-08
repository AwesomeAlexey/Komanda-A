cmake_minimum_required(VERSION 3.2)

project(butterfly_robot CXX)

find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_library(butterfly_robot STATIC
    inc/butterfly_robot/device_manager.h
    src/servo.cpp
    inc/butterfly_robot/servo.h
    src/camera.cpp
    inc/butterfly_robot/camera.h
    src/butterfly.cpp
    inc/butterfly_robot/butterfly.h
)
target_link_libraries(butterfly_robot PUBLIC
    cppmisc
    networking 
    Eigen3::Eigen 
    utils
)
target_include_directories(butterfly_robot PUBLIC inc)



# add_executable(test_servo_control test_servo_control.cpp)
# target_link_libraries(test_servo_control "${CMAKE_THREAD_LIBS}" butterfly)

# add_executable(test_servo_pos test_servo_pos.cpp)
# target_link_libraries(test_servo_pos "${CMAKE_THREAD_LIBS}" butterfly)

# add_executable(test_cam test_cam.cpp)
# target_link_libraries(test_cam "${CMAKE_THREAD_LIBS}" butterfly)
