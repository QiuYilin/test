#pragma once

#include <iostream>
#include <stdexcept>

#include "hash.hpp"

template <typename K, typename E>
class hashTable {
 public:
  hashTable(int theDivisor = 11);
  ~hashTable() { delete[] table; }

  bool empty() const { return dSize == 0; }
  int size() const { return dSize; }
  std::pair<const K, E>* find(const K&) const;
  void insert(const std::pair<const K, E>&);
  void output(ostream& out) const;

 protected:
  int search(const K&) const;
  std::pair<const K, E>** table;  // 散列表，储存std::pair<const K,E>* 的数组
  hash<K> hash;                   // 将K映射为非负整数,预处理
  int dSize;                      // 字典中的数对
  int divisor;                    // 哈希函数的除数
};

template <class K, class E>
hashTable<K, E>::hashTable(int theDivisor) {
  divisor = theDivisor;
  dSize = 0;

  table = new std::pair<const K, E>*[divisor];  // 确保下标不会越界
  for (int i = 0; i < divisor; i++) {
    table[i] = NULL;
  }
}

/// @brief 如果找到返回位置，如果没找到返回插入位置
/// @tparam K
/// @tparam E
/// @param theKey
/// @return
template <class K, class E>
int hashTable<K, E>::search(const K& theKey) const {
  int i = (int)hash(theKey) % divisor;  // 起始桶
  int j = i;
  // 三个终止条件①到达空桶 ②找到了关键字 ③回到起始桶
  while (table[j] != NULL && table[j]->first != theKey && j != i) {
    j = (j + 1) % divisor;  // next bucket
  }
  return j;
}

template <class K, class E>
std::pair<const K, E>* hashTable<K, E>::find(const K& theKey) const {
  int b = search(theKey);

  if (table[b] == NULL || table[b]->first != theKey) {
    return NULL;
  }

  return table[b];
}

template <class K, class E>
void hashTable<K, E>::insert(const std::pair<const K, E>& thePair) {
  int b = search(thePair.first);

  if (table[b] == NULL) {
    table[b] = new pair<const K, E>(thePair);  // 哈希值就是哈希表下标序号
    dSize++;
  } else {
    if (table[b]->first == thePair.first) {
      table[b]->second = thePair.second;
    } else {
      throw std::runtime_error("The hash table is full");
    }
  }
}