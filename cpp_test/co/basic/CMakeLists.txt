cmake_minimum_required(VERSION 3.21)
set(project_name "basic")

project(${project_name})

set(CMAKE_CXX_STANDARD 17)

add_executable(${project_name} main.cpp 
)

add_executable(basic1 basic1.cpp 
)

add_executable(basic2 basic2.cpp 
)

add_executable(basic3 basic3.cpp 
)

add_executable(thread_guard thread_guard.cpp 
)

add_executable(detach detach.cpp 
)

add_executable(param param.cpp 
)

add_executable(scoped_thread scoped_thread.cpp 
)

add_executable(joining_thread joining_thread.cpp 
)

add_executable(accumulate accumulate.cpp 
)

# add_executable(thread_id thread_id.cpp 
# )

add_executable(thread_safe_stack thread_safe_stack.cpp 
)

add_executable(lock lock.cpp)

add_executable(unique_lock unique_lock.cpp)

add_executable(condition_var condition_var.cpp)

add_executable(fine_grained_lock fine_grained_lock.cpp)

add_executable(call_once call_once.cpp)

add_executable(reader_writer_lock reader_writer_lock.cpp)

add_executable(async async.cpp)

add_executable(async_param async_param.cpp)

add_executable(packaged_task packaged_task.cpp)

add_executable(promise promise.cpp)

add_executable(time_point time_point.cpp)

add_executable(parallel_quick_sort parallel_quick_sort.cpp)

add_executable(multi_future multi_future.cpp)

add_executable(thread_local thread_local.cpp)

