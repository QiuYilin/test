#include <type_traits>

template <typename T>
typename std::enable_if_t<(sizeof(T) > 4)> foo() {}

template <typename T>
typename std::enable_if_t<(sizeof(T) > 4), T> foo2() {
  return T();
}

template <typename T, typename = std::enable_if_t<(sizeof(T) > 4), T>>
void foo() {}

template <typename T>
using EnableIfSizeGreater4 = std::enable_if_t<(sizeof(T) > 4)>;

template <typename T, typename = EnableIfSizeGreater4<T>>
void foo() {}