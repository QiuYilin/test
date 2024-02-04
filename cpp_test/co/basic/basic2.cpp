#include <iostream>
#include <thread>

void do_something() { std::cout << "do something 1\n"; }
void do_something_else() { std::cout << "do something 2\n"; }

class background_task {
 public:
  background_task() { std::cout << "constructor" << std::endl; }
  background_task(const background_task& s) {
    std::cout << "copy constructor" << std::endl;
  }
  void operator()() const {  // 使得类像函数一样调用的手段
    do_something();
    do_something_else();
  }

  void run() {
    do_something();
    do_something_else();
  }
};

int main() {
  background_task f;
  std::thread t(f);
  std::thread t2{background_task()};
  std::thread t3([]() {
    do_something();
    do_something_else();
  });
  try {
    std::cout << "do something in current thread" << std::endl;
  } catch (...) {
    t.join();
    t2.join();
    t3.join();
  }
  std::thread t4(&background_task::run,
                 &f);  // 传值会拷贝，深拷贝，分配了新的地址，值相同
  t.join();
  t2.join();
  t3.join();
  t4.join();
  if (t4.joinable()) {
    t4.join();
  }
  std::cout << "all finished" << std::endl;
  return 0;
}
