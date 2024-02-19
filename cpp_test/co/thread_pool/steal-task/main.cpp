#include <iostream>

#include "thread_pool.hpp"

thread_local work_stealing_queue* thread_pool::local_work_queue{};

thread_local unsigned thread_pool::my_index;

class Test {
 public:
  void print_one() { std::cout << "111" << std::endl; }
  void print_zero() { std::cout << "0" << std::endl; }
};
void print() { std::cout << "0" << std::endl; }
void print_n(int number) { std::cout << number << std::endl; }

int main() {
  thread_pool pool;
  Test test;
  auto f1 = pool.submit(&Test::print_one, &test);
  // auto f2 = pool.submit([&test]() { test.print_one(); });
  // int n = 3;
  // auto f3 = pool.submit(print_n, std::move(n));
  // auto f4 = pool.submit(print);
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}