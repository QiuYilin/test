cmake_minimum_required(VERSION 3.21)
set(project_name "fine_threadsafe_queue")

set(CMAKE_CXX_STANDARD 17)

project(${project_name})

set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

add_executable(${project_name} 
                            main.cpp 
                            threadsafe_queue.hpp
                            )
