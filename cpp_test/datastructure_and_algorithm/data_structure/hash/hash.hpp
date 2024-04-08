#pragma once
#include <iostream>
#include <string>

/// @brief 实现散列函数的类
/// @tparam K
template <typename K>
class hash;

template <>
class hash<std::string> {
 public:
  size_t operator()(const std::string theKey) {
    unsigned long hashValue = 0;
    int length = (int)theKey.length();
    for (int i = 0; i < length; i++) {
      hashValue = 5 * hashValue + theKey.at(i);
    }
    return size_t(hashValue);
  }
};

template <>
class hash<int> {
 public:
  size_t operator()(const int theKey) const { return size_t(theKey); }
};

template <>
class hash<long> {
 public:
  size_t operator()(const long theKey) const { return size_t(theKey); }
};