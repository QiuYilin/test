#include <algorithm>
#include <execution>
#include <iostream>
#include <vector>

int main() {
  std::vector<int> v(10);
  int count = 0;
  std::for_each(std::execution::seq, v.begin(), v.end(),
                [&](int& x) { x = ++count; });
  std::cout << "std::execution::seq" << std::endl;
  for (const auto x : v) {
    std::cout << x << std::endl;
  }
  std::cout << "std::execution::par" << std::endl;
  std::for_each(std::execution::par, v.begin(), v.end(), [](auto& x) { ++x; });
  for (const auto x : v) {
    std::cout << x << std::endl;
  }
  return 0;
}