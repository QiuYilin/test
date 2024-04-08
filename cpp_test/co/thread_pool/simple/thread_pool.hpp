#pragma once

#include <atomic>
#include <functional>
#include <thread>
#include <vector>

#include "join_threads.hpp"
#include "threadsafe_queue.hpp"

class thread_pool {
  threadsafe_queue<std::function<void()>>
      work_queue;  // 管理任务队列的线程安全队列，只考虑简单函数，不需要返回值
  // 有返回值怎么办
  std::vector<std::thread> threads;  // 工作线程
  join_threads joiner;               // RAII

  void worker_thread() {  // 执行线程的函数
    while (!done) {
      std::function<void()> task;
      if (work_queue.try_pop(task)) {
        task();
      } else {
        std::this_thread::yield();
      }
    }
  }

 public:
  thread_pool()
      : done(false),
        joiner(
            threads) {  // 运行固定数量的线程，这些线程从一开始就去尝试从队列拿任务
    // 必须按照done worker_queue threads的顺序析构
    unsigned const thread_count =
        std::thread::hardware_concurrency();  // 获取硬件支持多少个并发线程
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

  template <typename Callable, typename... Args>
  void submit(Callable&& f,
              Args&&... args) {  // 将函数包装成std::function<void()>实例
    auto func = [f, args...]() mutable {
      std::invoke(std::forward<Callable>(f), std::forward<Args>(args)...);
    };
    work_queue.push(func);
  }

  std::atomic_bool done;
};