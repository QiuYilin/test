#include <iostream>
#include <thread>

class thread_guard {
  std::thread& _t;

 public:
  explicit thread_guard(std::thread& t_) : _t(t_) {}

  ~thread_guard() {
    if (_t.joinable()) {
      _t.join();
    }
  }

  thread_guard(thread_guard const&) = delete;
  thread_guard& operator=(thread_guard const&) = delete;
};

void do_something(int i) { std::cout << "do something " << i << std::endl; }

struct func {
  int& i;
  func(int& i_) : i(i_) {}
  void operator()() {
    for (unsigned j = 0; j < 1000000; ++j) {
      do_something(i);
    }
  }
};

int main() {
  int some_local_state = 0;
  func my_func(some_local_state);
  std::thread t(my_func);
  thread_guard g(t);
  std::cout << "in main thread" << std::endl;
}
