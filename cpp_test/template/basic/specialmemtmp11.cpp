#include "specialmemtmp11.hpp"

int main() {
  std::string s = "sname";
  Person p1(s);
  Person p2("tmp");
  Person p3(p1);
  Person p4(std::move(p1));

  Person_ p5(s);
  Person_ p6("tmp");

  PersonT p7("tmp");
  //   PersonT p8(p7);
  return 0;
}