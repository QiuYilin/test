cmake_minimum_required(VERSION 3.11)

set(project_name "default_func")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
)

add_executable(non_type_param non_type_param.cpp 
)

add_executable(foreach foreach.hpp  foreach.cpp 
)

add_executable(invoke invoke.cpp 
)

add_executable(function function.cpp 
)

add_executable(dynamic_memory_alloc dynamic_memory_alloc.cpp 
)

add_executable(compare compare.cpp 
)