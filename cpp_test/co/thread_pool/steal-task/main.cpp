#include <iostream>

#include "thread_pool.hpp"

thread_local work_stealing_queue* thread_pool::local_work_queue{};

thread_local unsigned thread_pool::my_index;

class Test {
 public:
  int print_one() {
    std::cout << "111" << std::endl;
    return 1;
  }
  void print_zero() { std::cout << "0" << std::endl; }
};
void print() { std::cout << "0" << std::endl; }
void print_n(int number) { std::cout << number << std::endl; }

int main() {
  thread_pool pool;
  Test test;
  auto f1 = pool.submit(&Test::print_one, &test);
  std::this_thread::sleep_for(std::chrono::seconds(10));
  return 0;
}