cmake_minimum_required(VERSION 3.11)

set(project_name "stack")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
                               Stack1.hpp
                               Stack2.hpp
                               Stack3.hpp
                               Stacknontype.hpp
)

add_executable("stack_auto"    stackauto.hpp
                               stackauto.cpp
)

add_executable("varprint"      varprint.cpp
)

add_executable("varbase"      varbase.cpp
)
