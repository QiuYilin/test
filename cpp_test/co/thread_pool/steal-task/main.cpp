#include <iostream>

#include "thread_pool.hpp"

thread_local work_stealing_queue* thread_pool::local_work_queue{};

thread_local unsigned thread_pool::my_index;

void print_one() { std::cout << "1" << std::endl; }

void print_zero() { std::cout << "0" << std::endl; }

int main() {
  thread_pool pool;
  auto f1 = pool.submit(print_one);
  auto f2 = pool.submit(print_zero);

  return 0;
}