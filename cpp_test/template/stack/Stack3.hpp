#include <cassert>
#include <string>
#include <vector>

template <typename T, typename Cont = std::vector<T>>
class Stack {
 private:
  Cont elems;

 public:
  Stack(T elem) : elems({std::move(elem)}) {}
  void push(T const& elem);
  void pop();
  T const& top() const;
  bool empty() const { return elems.empty(); }
};

Stack(char const*) -> Stack<std::string>;

template <typename T, typename Cont>
void Stack<T, Cont>::push(T const& elem) {
  elems.push_back(elem);
}

template <typename T, typename Cont>
void Stack<T, Cont>::pop() {
  assert(!elems.empty());
  elems.pop_back();
}

template <typename T, typename Cont>
T const& Stack<T, Cont>::top() const {
  assert(!elems.empty());
  return elems.back();
}
