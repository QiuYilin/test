#include <iostream>

#include "thread_pool.hpp"

void print_one() { std::cout << "1" << std::endl; }

void print_zero() { std::cout << "0" << std::endl; }

int print_n(int n) {
  std::cout << n << std::endl;
  return n;
}

class Test {
 public:
  int sum(int a, int b) {
    // std::cout << "a+b" << a + b << std::endl;
    return a + b;
  }
};

int main() {
  thread_pool pool;
  Test test;
  // pool.submit(print_n, 5);
  auto future = pool.submit(&Test::sum, &test, 3, 4);
  std::cout << "future" << future.get() << std::endl;
  while (pool.done == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return 0;
}