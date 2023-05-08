cmake_minimum_required(VERSION 3.2)

project(sliding_controller_test)
enable_testing()
find_package(GTest REQUIRED)

add_executable(sliding_controller_test
    sliding_controller_test.cpp
    ../src/sliding_controller.cpp
    ../inc/control_system/sliding_controller.h
)

target_include_directories(sliding_controller_test PRIVATE "../inc")
target_link_libraries(sliding_controller_test butterfly_robot utils ${GTEST_LIBRARIES})
add_test(NAME sliding_controller_test COMMAND sliding_controller_test "${CMAKE_SOURCE_DIR}/configs/sliding_controller.json")
