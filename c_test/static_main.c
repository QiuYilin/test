#include <stdio.h>

// 静态局部变量
void test() {
  static int count = 0;
  count++;
  printf("count = %d\n", count);
}

// 静态全局变量
static int global_count = 0;

// 静态函数
static void static_func() { printf("This is a static function.\n"); }

int main() {
  // 调用静态局部变量
  test();
  test();
  test();

  // 调用静态全局变量
  global_count++;
  printf("global_count = %d\n", global_count);

  // 调用静态函数
  static_func();

  return 0;
}
