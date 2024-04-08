#include <iostream>
int factorial(int n) {
  if (n <= 1) {
    return 1;
  } else {
    return n * factorial(n - 1);
  }
}

template <typename T>
T sum(T a[], int n) {
  T theSum = 0;
  for (int i = 0; i < n; i++) {
    theSum += a[i];
  }
  return theSum;
}

template <typename T>
T rSum(T a[], int n) {
  if (n > 0) {
    return rsum(a, n - 1) + a[n - 1];
  }
  return 0;
}

template <typename T>
T StlSum(T a[], int n) {
  T theSum = 0;
  return accumlate(a, a + n, theSum);
}

template <typename T>
void permutations(T list[], int k, int m) {  // 生成list[k:m]的所有排列
  if (k == m) {  // list[k:m]只有一个排列，输出
    std::copy(list, list + m + 1, std::ostream_iterator<T>(std::cout, ""));
    std::cout << std::endl;
  } else {  // 多于一个排列，递归生成这些排列
    for (int i = k; i <= m; i++) {
      swap(list[k], list[i]);
      permutations(list, k + 1, m);
      swap(list[k], list[i]);
    }
  }
}

int main() {
  std::cout << "result " << factorial(25) << std::endl;

  return 0;
}