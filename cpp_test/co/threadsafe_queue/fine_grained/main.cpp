#include <iostream>

#include "threadsafe_queue.hpp"

int main() {
  threadsafe_queue<float> queue;
  queue.push(10);
  queue.push(100);
  std::cout << *queue.wait_and_pop();
  std::cout << *queue.wait_and_pop();
  return 0;
}