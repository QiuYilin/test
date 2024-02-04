#include <iostream>
#include <thread>

void do_something(int& i) {
  ++i;
  std::cout << i << std::endl;
}

struct func {
  int& i;
  func(int& i_) : i(i_) {}
  void operator()() {
    for (unsigned j = 0; j < 1000; ++j) {
      do_something(i);
    }
  }
};

// RAII 包装成对象
class thread_guard {
  std::thread& t;

 public:
  explicit thread_guard(std::thread& t_) : t(t_) {}
  ~thread_guard() {
    if (t.joinable()) {
      t.join();
    }
  }
  thread_guard(thread_guard const&) = delete;
  thread_guard& operator=(thread_guard const&) = delete;
};

void f() {
  int some_local_state = 0;
  int some_local_state2 = 0;
  func my_func(some_local_state);
  func my_func2(some_local_state2);
  std::thread my_thread(my_func);
  std::thread my_thread2(my_func2);
  try {
    std::cout << "in main thread 1" << std::endl;
  } catch (...) {
    my_thread.join();
    throw;
  }
  thread_guard g(my_thread2);
  my_thread.join();
  std::cout << "in main thread 2" << std::endl;
}

int main() { f(); }