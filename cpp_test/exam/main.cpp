#include <iostream>
using namespace std;
class test {
 public:
  static int i;
  int j;
  test(int a) : i(1), j(a) {}
  void func1();
  static void func2();
};

void test::func1() { cout << i << ", " << j << endl; }

void test::func2() { cout << i << ", " << j << endl; }

int main() {
  test t(2);
  t.func1();
  t.func1();
  return 0;
}