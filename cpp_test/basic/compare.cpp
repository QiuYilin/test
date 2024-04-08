#include <iostream>
struct A {
  int m;
};
int main() {
  A a, b;
  a.m = 1;
  b.m = 1;
  if (a == b) {
    std::cout << "equal" << std::endl;
  }
  return 0;
}