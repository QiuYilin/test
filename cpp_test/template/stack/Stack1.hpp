#include <cassert>
#include <stack>
#include <string>
#include <vector>

template <typename T>
class Stack {
 private:
  std::vector<T> elems;

 public:
  void push(T const& elem);
  void pop();
  T const& top() const;
  bool empty() const { return elems.empty(); }

  void printOn(std::ostream& strm) const {
    for (T const& elem : elems) {
      strm << elem << ' ';
    }
  }

  friend std::ostream& operator<<(std::ostream& strm, Stack<T> const& s) {
    s.printOn(strm);
    return strm;
  }
};

template <typename T>
void Stack<T>::push(T const& elem) {
  elems.push_back(elem);
}

template <typename T>
void Stack<T>::pop() {
  assert(!elems.empty());
  return elems.pop_back();
}

template <typename T>
T const& Stack<T>::top() const {
  assert(!elems.empty());
  return elems.back();
}

template <>
class Stack<std::string> {
 private:
  std::deque<std::string> elems;

 public:
  void push(std::string const&);
  void pop();
  std::string const& top() const;
  bool empty() const { return elems.empty(); }
};

void Stack<std::string>::push(std::string const& elem) {
  elems.push_back(elem);
}

void Stack<std::string>::pop() {
  assert(!elems.empty());
  elems.pop_back();
}

std::string const& Stack<std::string>::top() const {
  assert(!elems.empty());
  return elems.back();
}

// 部分特化
template <typename T>
class Stack<T*> {
 private:
  std::vector<T*> elems;

 public:
  void push(T*);
  T* pop();
  T* top() const;
  bool empty() const { return elems.empty(); }
};

template <typename T>
void Stack<T*>::push(T* elem) {
  elems.push_back(elem);
}

template <typename T>
T* Stack<T*>::pop() {
  assert(!elems.empty());
}

template <typename T>
T* Stack<T*>::top() const {
  assert(!elems.empty());
  return elems.back();
  elems.pop_back();
  return p;
}