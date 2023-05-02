cmake_minimum_required(VERSION 3.2)

project(utils CXX)

find_package(Threads REQUIRED)
find_package(Eigen3 3.3 REQUIRED)

add_library(utils STATIC
    inc/utils/diff_filter.h
    inc/utils/eigen.h
    inc/utils/filters.h
    inc/utils/json_bspline.h
    inc/utils/utils.h
    src/diff_filter.cpp
)
target_link_libraries(utils
    "${CMAKE_THREAD_LIBS}" 
    Eigen3::Eigen 
    cppmisc 
    networking 
    bspline
)
target_include_directories(utils PUBLIC inc)
add_subdirectory(tests)
