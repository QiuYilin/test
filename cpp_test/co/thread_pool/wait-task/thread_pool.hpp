#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <memory>
#include <thread>
#include <vector>

#include "threadsafe_queue.hpp"

class
    function_wrapper {  // 这里主要是因为std::packaged_task的实例是void(),所以没有做成可变参数模板也能使用
  struct impl_base {
    virtual void call() = 0;
    virtual ~impl_base() {}
  };
  std::unique_ptr<impl_base> impl;
  template <typename F>
  struct impl_type : impl_base {
    F f;
    impl_type(F&& f_) : f(std::move(f_)) {}
    void call() { f(); }
  };

 public:
  template <typename F>
  function_wrapper(F&& f) : impl(new impl_type<F>(std::move(f))) {}

  void operator()() { impl->call(); }

  function_wrapper() = default;

  function_wrapper(function_wrapper&& other) : impl(std::move(other.impl)) {}

  function_wrapper& operator=(function_wrapper&& other) {
    impl = std::move(other.impl);
    return *this;
  }

  function_wrapper(const function_wrapper&) = delete;
  function_wrapper(function_wrapper&) = delete;
  function_wrapper& operator=(const function_wrapper&) = delete;
};

struct join_threads {
  join_threads(std::vector<std::thread>&) {}
};

class thread_pool {
  threadsafe_queue<function_wrapper> work_queue;
  std::vector<std::thread> threads;
  join_threads joiner;

  void worker_thread() {
    while (!done) {
      function_wrapper task;
      if (work_queue.try_pop(task)) {
        task();
      } else {
        std::this_thread::yield();
      }
    }
  }

 public:
  thread_pool() : done(false), joiner(threads) {
    unsigned const thread_count = std::thread::hardware_concurrency();
    try {
      for (unsigned i = 0; i < thread_count; ++i) {
        threads.push_back(std::thread(&thread_pool::worker_thread, this));
      }
    } catch (...) {
      done = true;
      throw;
    }
  }

  ~thread_pool() { done = true; }

  /// @brief 返回std::future<>保存任务的返回值，允许调用者等待任务完全结束
  /// @tparam FunctionType
  /// @param f
  /// @return
  template <typename Callable, typename... Args>
  decltype(auto) submit(Callable f, Args... args) {
    typedef typename std::invoke_result_t<Callable, Args...> result_type;

    std::packaged_task<result_type()> task([f, args...]() mutable {
      return std::invoke(std::forward<Callable>(f),
                         std::forward<Args>(args)...);
    });
    std::future<result_type> res(task.get_future());
    work_queue.push(std::move(task));
    return res;
  }

  std::atomic_bool done;
};