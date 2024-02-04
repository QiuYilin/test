// CPP Program to demonstrate the use of copy constructor
#include <stdio.h>

#include <iostream>

using namespace std;

class X {
 public:
  // Constructor
  X() {}
  // Copy Constructor
  X(const X& s) { cout << " Copy constructor has been called " << endl; }
  // Assignment operator
  X& operator=(X const& rhs) {
    cout << " Assignment operator has been called " << endl;
    return *this;
  }
  // Move constructor
  X(X&& s) { std::cout << " Move constructor called" << std::endl; }
  // Move assignment operator
  X& operator=(X&& rhs) {
    cout << " Move assignment operator has been called " << endl;
    return *this;
  }
};

X func() {
  X obj;
  return obj;
}

void func1(X x) { return; }

int main() {
  X obj1;
  X obj2 = obj1;    // 会发生拷贝构造
  X obj3{obj1};     // 会发生拷贝构造
  X obj4 = func();  // 不一定发生拷贝构造
  func1(obj1);      // 会发生拷贝构造
  X obj5;
  obj5 = obj1;              // 会发生赋值操作
  X obj6(std::move(obj1));  // 会发生移动构造
  X obj7;
  obj7 = std::move(obj6);  // 会发生移动赋值运算符
  return 0;
}