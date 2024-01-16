set(project_name "simple")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
                            threadsafe_queue.hpp
                            thread_pool.hpp)
