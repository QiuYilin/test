#include <iostream>

#include "thread_pool.hpp"

void print_one() { std::cout << "1" << std::endl; }

void print_zero() { std::cout << "0" << std::endl; }

int main() {
  thread_pool pool;
  pool.submit(print_one);
  pool.submit(print_zero);
  while (pool.done == false)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  return 0;
}