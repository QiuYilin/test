#include <iostream>
void print() {}

template <typename T, typename... Types>
void print(T firstArg, Types... args) {
  std::cout << firstArg << '\n';
  std::cout << "sizeof ..." << sizeof...(Types) << '\n';
  std::cout << "sizeof ..." << sizeof...(args) << '\n';
  print(args...);
}

int main() {
  std::string s("world");
  print(7.5, "hello", s);
  return 0;
}