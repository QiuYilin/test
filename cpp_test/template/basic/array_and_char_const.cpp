#include <iostream>

// 未知长度 未知类型的数组
template <typename T, int N, int M>
bool less(T (&a)[N], T (&b)[M]) {
  for (int i = 0; i < N && i < M; ++i) {
    if (a[i] < b[i]) {
      return true;
    }
    if (b[i] < a[i]) {
      return false;
    }
  }
  return N < M;
}

// 未知长度的字符串常量 本质上也是数组
template <int N, int M>
bool less(char const (&a)[N], char const (&b)[M]) {
  for (int i = 0; i < N && i < M; ++i) {
    if (a[i] < b[i]) {
      return true;
    }
    if (b[i] < a[i]) {
      return false;
    }
  }
  return N < M;
}

template <typename T>
struct MyClass;  // 主模板
template <typename T, std::size_t SZ>
struct MyClass<T[SZ]> {  // 已知边界的数组偏特化
  static void print() { std::cout << "print() for T[" << SZ << "]\n"; }
};
template <typename T, std::size_t SZ>
struct MyClass<T (&)[SZ]> {  // 已知边界的数组的引用的偏特化
  static void print() { std::cout << "print() for T(&)[" << SZ << "]\n"; }
};
template <typename T>
struct MyClass<T[]> {  // 未知边界的数组的偏特化
  static void print() { std::cout << "print() for T[]\n"; }
};

template <typename T>  // 未知边界的数组的引用的偏特化
struct MyClass<T (&)[]> {
  static void print() { std::cout << "print() for T(&)[]\n"; }
};

template <typename T>
struct MyClass<T*> {  // 指针的偏特化
  static void print() { std::cout << "print() for T*\n"; }
};

template <typename T1, typename T2, typename T3>
void foo(int a1[7], int a2[],  // 指针
         int (&a3)[42],        // 已知边界的数组的引用
         int (&x0)[],          // 未知边界的数组的引用
         T1 x1,                // 值退化
         T2& x2, T3&& x3)      // 引用传递
{
  MyClass<decltype(a1)>::print();  // uses MyClass<T*>
  MyClass<decltype(a2)>::print();  // uses MyClass<T*> a1, a2 退化成 指针
  MyClass<decltype(a3)>::print();  // uses MyClass<T(&)[SZ]>
  MyClass<decltype(x0)>::print();  // uses MyClass<T(&)[]>
  MyClass<decltype(x1)>::print();  // uses MyClass<T*>
  MyClass<decltype(x2)>::print();  // uses MyClass<T(&)[]>
  MyClass<decltype(x3)>::print();  // uses MyClass<T(&)[]> // 万能引
                                   // 用，引用折叠
}

int main() {
  int x[] = {1, 2, 3};
  int y[] = {1, 2, 3, 4, 5};
  std::cout << less(x, y) << std::endl;

  const char s1[] = "hello";
  const char s2[] = "hi";
  std::cout << less(s1, s2) << std::endl;
  std::cout << less<const char>(s1, s2) << std::endl;

  int a[42];
  MyClass<decltype(a)>::print();
  extern int x1[];
  MyClass<decltype(x)>::print();
  foo(a, a, a, x1, x1, x1, x1);
  return 0;
}

int x1[] = {0, 8, 15};