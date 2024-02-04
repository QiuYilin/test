#include <iostream>
#include <thread>

void f(int i, std::string const& s) { std::cout << i << " " << s << std::endl; }

void oops(int some_param) {
  char buffer[1024];
  sprintf(buffer, "%i", some_param);
  std::thread t(f, 3, std::string(buffer));
  t.detach();
}

int main() {
  // std::thread t(f, 3, "hello");
  oops(3);
  // t.join();
}