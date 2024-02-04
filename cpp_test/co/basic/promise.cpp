#include <future>
#include <iostream>
#include <thread>

void async_task(std::promise<int> promise_obj) {
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  promise_obj.set_value(21);
}

int main() {
  std::promise<int> promise_obj;
  std::future<int> future_obj = promise_obj.get_future();

  std::thread async_thread(async_task, std::move(promise_obj));

  std::cout << "Waiting for async result..." << std::endl;
  int result = future_obj.get();
  std::cout << "Async result is " << result << std::endl;

  async_thread.join();
  std::chrono::duration<double, std::ratio<1, 1000>> d;
  return 0;
}