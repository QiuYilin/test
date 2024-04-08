#include <iostream>
#include <stdexcept>

// 动态分配两个维度的数组
// template <typename T>
// bool make2dArray(T **&x, int numberOfRows, int numberOfColumns) {
//   try {
//     // 创建行指针
//     x = new T *[numberOfRows];
//     // 为每一行分配空间
//     for (int i = 0; i < numberOfRows; i++) {
//       x[i] = new int[numberOfColumns];
//     }
//     return true;
//   } catch (std::bad_alloc) {
//     return false;
//   }
// }

template <typename T>
void make2dArray(T **&x, int numberOfRows, int numberOfColumns) {
  // 创建行指针
  x = new T *[numberOfRows];
  // 为每一行分配空间
  for (int i = 0; i < numberOfRows; i++) {
    x[i] = new T[numberOfColumns];
  }
}

template <typename T>
void delete2dArray(T **&x, int numberOfRows) {
  for (int i = 0; i < numberOfRows; i++) {
    delete[] x[i];
  }
  delete[] x;
  x = NULL;
}

int main() {
  int *y = new int[10];

  char(*c)[5];
  try {
    int n = 10;
    c = new char[n][5];
    std::cout << "c " << c << std::endl;
  } catch (std::bad_alloc) {
    std::cout << "Out of Memory" << std::endl;
    exit(1);
  }

  int **array;
  try {
    make2dArray(array, 10, 10);
  } catch (std::bad_alloc) {
    std::cout << "Out of Memory" << std::endl;
    exit(1);
  }

  array[4][4] = 10;
  return 0;
}
