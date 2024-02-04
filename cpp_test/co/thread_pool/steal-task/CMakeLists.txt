cmake_minimum_required(VERSION 3.21)
set(project_name "task-steal-threadpool")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
                            join_threads.hpp
                            function_wrapper.hpp
                            threadsafe_queue.hpp
                            work_stealing_queue.hpp
                            thread_pool.hpp)
