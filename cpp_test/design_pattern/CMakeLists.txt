cmake_minimum_required(VERSION 3.11)

set(project_name "default_func")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable( proxy Structural/proxy.cpp 
)


add_executable( composite Structural/composite.cpp 
)

add_executable( template_method Behavioral/template_method.cpp 
)

add_executable( strategy_dynamic Behavioral/strategy/strategy_dynamic.cpp 
)

add_executable( strategy_static Behavioral/strategy/strategy_static.cpp 
)

add_executable( observer1 Behavioral/observer1.cpp 
)