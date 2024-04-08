#include "../data_structure/linearList/list.hpp"

#include <iostream>
#include <list>

int main() {
  List<int> list;
  list.push_back(1);
  list.push_back(2);
  list.push_back(3);
  list.push_back(4);

  list.insert(++list.begin(), 5);

  for (auto it : list) {
    std::cout << it << " ";
  }

  std::cout << std::endl;

  auto it = std::find(list.begin(), list.end(), 2);
  if (it != list.end()) {
    list.insert(it, 6);
  }

  for (auto it : list) {
    std::cout << it << " ";
  }

  std::cout << std::endl;

  std::list<int> list2;
  list2.push_back(1);
  list2.push_back(2);
  list2.push_back(3);
  list2.push_back(4);

  auto it1 = std::find(list2.begin(), list2.end(), 2);
  if (it1 != list2.end()) {
    list2.insert(++it1, 5);
  }

    // list2.insert(list2.begin()+3, 6);

  for (const auto& element : list2) {
    std::cout << element << " ";
  }
  std::cout << std::endl;

  return 0;
}