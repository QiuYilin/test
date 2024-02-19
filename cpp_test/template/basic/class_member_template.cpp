template <typename T>
class Stack {
 private:
  std::deque<T> elems;

 public:
  void push(T const&);
  void pop();
  T const& top() const;
  bool empty() const { return elems.empty(); }
  template <typename T2>
  Stack& operator=(Stack<T2> const&);
};

template <typename T>
template <typename T2>
Stack<T>& Stack<T>::operator=(Stack<T2> const& op2) {
  Stack<T2> tmp(op2);
  elems.clear();
  while (!tmp.empty()) {
    elems.push_front(tmp.top());
    tmp.pop();
  }
  return *this;
}