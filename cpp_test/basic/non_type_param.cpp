#include <iostream>

template <typename T, const char* name>
class Message {
  T value;

 public:
  Message(T value_) : value(value_) {}
  void print() {
    std::cout << "name " << name << ' value ' << value << std::endl;
  }
};

template <const char* name>
class Message2 {
 public:
  void print() { std::cout << name << std::endl; }
};

int main() {
  static char const s17[] = "hi";
  Message<int, s17>
      m17;  // no matching constructor for initialization of 'Message<int,s17>'
  m17.print();

  static char const s172[] = "hi";
  Message2<s172> m172;
  m172.print();
}
