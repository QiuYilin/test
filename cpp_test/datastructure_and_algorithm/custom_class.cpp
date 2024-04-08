#include "currency1.hpp"

int main() {
  currency g, h(plus, 3, 50), i, j;

  g.setValue(minus, 2, 25);
  i.setValue(-6.45);

  j = h.add(g);
  h.output();
  std::cout << "+";
  g.output();
  std::cout << "=";
  j.output();
  std::cout << std::endl;

  j = i.add(g).add(h);
  j = i.increment(g).add(h);

  std::cout << "Attempting to initialize with cents = 152" << std::endl;
  try {
    i.setValue(plus, 3, 152);
  } catch (illegalParameterValue e) {
    std::cout << "Caught thrown exception" << std::endl;
    e.outputMessage();
  }

  return 0;
}