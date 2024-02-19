cmake_minimum_required(VERSION 3.11)

set(project_name "basic")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable("array_and_char_const" array_and_char_const.cpp 
)

add_executable("move" move.cpp 
)

add_executable("specialmemtmp11" specialmemtmp11.cpp  specialmemtmp11.hpp 
)

add_executable("enable_if" enable_if.cpp 
)

add_executable("practice" practice.hpp practice.cpp
)

add_executable("error_test" error_test.cpp
)