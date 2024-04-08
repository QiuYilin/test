#include <future>
#include <iostream>

#include "thread_pool.hpp"

// void print_one() { std::cout << "1" << std::endl; }

// void print_zero() { std::cout << "0" << std::endl; }

class Test {
 public:
  int print_n(int n) {
    std::cout << "n " << n << std::endl;
    return n;
  }
};

int main() {
  thread_pool pool;
  Test test;
  pool.submit(&Test::print_n, &test, 5);
  while (pool.done == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return 0;
}