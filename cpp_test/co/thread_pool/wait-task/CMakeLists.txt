set(project_name "wait-for-task_threadpool")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
                            threadsafe_queue.hpp
                            thread_pool.hpp)
