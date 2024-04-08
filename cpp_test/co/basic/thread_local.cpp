#include <iostream>
#include <mutex>
#include <thread>

std::mutex cout_mutex;

thread_local int x = 1;

void thread_func(const std::string& thread_name) {
  for (int i = 0; i < 3; ++i) {
    x++;
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << "thread[" << thread_name << "]: x = " << x << std::endl;
  }
  return;
}

void thread_func2(const std::string& thread_name) {
  for (int i = 0; i < 3; ++i) {
    thread_local int y = 1;
    y++;
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << "thread[" << thread_name << "]: x = " << y << std::endl;
  }
  return;
}

class B {
 public:
  B() {
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << "create B" << std::endl;
  }
  ~B() {}
  thread_local static int b_key;
  // thread_local int b_key;
  int b_value = 24;
  static int b_static;
};

// thread_local int B::b_key = 12;
// int B::b_static = 36;

void thread_func3(const std::string& thread_name) {
  B b;
  for (int i = 0; i < 3; ++i) {
    b.b_key--;
    b.b_value--;
    // b.b_static--;  // not thread safe
    std::lock_guard<std::mutex> lock(cout_mutex);
    std::cout << "thread[" << thread_name << "]: b_key:"
              << b.b_key
              //<< ", b_value:" << b.b_value << ", b_static:" << b.b_static
              << std::endl;
    std::cout << "thread[" << thread_name << "]: B::key:"
              << B::b_key
              //<< ", b_value:" << b.b_value << ", b_static: " << B::b_static
              << std::endl;
  }
  return;
}

int main() {
  std::thread t1(thread_func, "t1");
  std::thread t2(thread_func, "t2");
  std::thread t3(thread_func2, "t1");
  std::thread t4(thread_func2, "t2");
  std::thread t5(thread_func3, "t2");
  t1.join();
  t2.join();
  t3.join();
  t4.join();
  t5.join();
  return 0;
}