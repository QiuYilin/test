#pragma once
template <typename Iter, typename Callable>
void foreach (Iter current, Iter end, Callable op) {
  while (current != end) {  // as long as not reached the end
    op(*current);           //  call passed operator for current element
    ++current;              //  and move iterator to next element
  }
}

#include <functional>
#include <utility>
template <typename Iter, typename Callable, typename... Args>
void foreach2(Iter current, Iter end, Callable op, Args const&... args) {
  while (current != end) {  // as long as not reached the end
    std::invoke(op, args...,
                *current);  //  call passed operator for current element
    ++current;              //  and move iterator to next element
  }
}