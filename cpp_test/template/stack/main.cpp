#include <iostream>
#include <string>

// stack1
//  #include "stack1.hpp"

// int main() {
//   Stack<int> intStack;
//   Stack<std::string> stringStack;
//   intStack.push(7);
//   std::cout << intStack.top() << '\n';

//   stringStack.push("hello");
//   std::cout << stringStack.top() << "\n";
//   stringStack.pop();

//   Stack<std::pair<int, int>> ps;
//   ps.push({4, 5});                       // OK
//   ps.push({6, 7});                       // OK
//   std::cout << ps.top().first << '\n';   // OK
//   std::cout << ps.top().second << '\n';  // OK
//   // ps.printOn(std::cout);

//   Stack<int *> ptrStack;
//   ptrStack.push(new int{42});
//   std::cout << *ptrStack.top() << '\n';
//   delete ptrStack.pop();
// }
// stack1

// stack2
//  #include <deque>
//  #include <iostream>

// #include "stack2.hpp"

// int main() {
//   Stack<int> intStack;
//   Stack<double, std::deque<double>> dblStack;

//   intStack.push(7);
//   std::cout << intStack.top() << std::endl;
//   intStack.pop();

//   dblStack.push(42.42);
//   std::cout << dblStack.top() << std::endl;
//   dblStack.pop();
// }
// stack2

// stack3
//  #include "stack3.hpp"

// int main() {
//   Stack stringStack{"bottom"};
//   Stack stack3{stringStack};
// }
// stack3

#include "Stacknontype.hpp"

int main() {
  Stack<int, 20> int20Stack;           // stack of up to 20 ints
  Stack<int, 40> int40Stack;           // stack of up to 40 ints
  Stack<std::string, 40> stringStack;  // stack of up to 40 strings

  // manipulate stack of up to 20 ints
  int20Stack.push(7);
  std::cout << int20Stack.top() << std::endl;
  int20Stack.pop();

  // manipulate stack of up to 40 strings
  stringStack.push("hello");
  std::cout << stringStack.top() << std::endl;
  stringStack.pop();
}