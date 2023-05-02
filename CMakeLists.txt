cmake_minimum_required(VERSION 3.2)

project(butterfly_control_system)

set(CMAKE_CXX_FLAGS_RELEASE "-march=native -mtune=native -Ofast")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(3rd_party/bsplines)
add_subdirectory(3rd_party/cppmisc)
add_subdirectory(3rd_party/cppnetworking)
add_subdirectory(src)
