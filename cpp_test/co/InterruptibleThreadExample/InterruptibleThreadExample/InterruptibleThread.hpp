#pragma once
#ifndef INTERRUPTIBLE_THREAD
#define INTERRUPTIBLE_THREAD

#include <atomic>
#include <exception>
#include <future>
#include <iostream>
#include <stdexcept>
#include <thread>

extern thread_local bool done;

class thread_interrupted : public std::exception {
 public:
  bool t_interrupted;
  thread_interrupted() : t_interrupted{true} {}
};

namespace {

void interruption_point()  // 断点
                           // 可以访问thread_local的变量，在线程运行时对变量进行设置
{
  if (this_thread_interrupt_flag.is_set())  // 检查当前运行线程的数据结构
  {
    std::cout << "The thread with id: " << std::this_thread::get_id()
              << " has been interrupted!!!\n"
              << std::flush;
    throw thread_interrupted();
  }
}

class interrupt_flag {
 public:
  std::atomic<bool> flag;
  void set();
  bool is_set() const;
};

void interrupt_flag::set() { flag.store(true, std::memory_order_relaxed); }

bool interrupt_flag::is_set() const {
  return flag.load(std::memory_order_relaxed);
}

thread_local interrupt_flag this_thread_interrupt_flag;
}  // namespace

class InterruptibleThread {
 public:
  template <typename FunctionType>
  InterruptibleThread(FunctionType f) {
    std::promise<interrupt_flag*> p;  // 线程持有f的副本和本地pomise变量
    internal_thread = std::thread([f, &p] {  //
      p.set_value(&this_thread_interrupt_flag);  // 设置诺值变量的值到this_thread_interrupt_flag地址
                                                 // 为调用f()做准备
      try {
        f();
      } catch (thread_interrupted const& e) {
        done = e.t_interrupted;
      }
    });
    flag = p.get_future().get();  // ！！！！！！！！！！核心内容
                                  // 这段有啥用
  }
  void join();
  void detach();
  bool joinable() const;
  void interrupt();  // 可中断的线程应当比一般的thread多的接口
 private:
  std::thread internal_thread;
  interrupt_flag* flag;
};

#endif  // !INTERRUPTIBLE_THREAD
