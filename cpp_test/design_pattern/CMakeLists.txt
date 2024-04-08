cmake_minimum_required(VERSION 3.11)

set(project_name "default_func")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable( proxy Structural/proxy.cpp 
)


add_executable( composite Structural/composite.cpp 
)