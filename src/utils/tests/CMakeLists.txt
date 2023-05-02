cmake_minimum_required(VERSION 3.2)
project(utils_tests)
add_executable(test_json test_json.cpp)
target_link_libraries(test_json "${CMAKE_THREAD_LIBS}" utils)
