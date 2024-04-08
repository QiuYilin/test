
#pragma once
#include <sstream>
#include <stdexcept>

template <typename T>
class arrayQueue : public queue<T> {
 public:
  arrayQueue(int initialCapacity = 10);
  ~arrayQueue() { delete[] queue; }
  bool empty() const { return theFront == theBack; }
  int size() const { return (theBack - theFront + arrayLength) % arrayLength; }
  T& front() {
    if (theFront == theBack) {
      throw std::runtime_error("queue empty");
    }
    return queue[(theFront + 1) % arrayLength];
  }
  T& back() {
    if (theFront == theBack) {
      throw std::runtime_error("queue empty");
    }
    return queue[theBack];
  }
  void pop() {
    if (theFront == theBack) {
      throw std::runtime_error("queue empty");
    }
    theFront = (theFront + 1) % arrayLength;
    queue[theFront].~T();
  }
  void push(const T& theElement);

 private:
  int theFront;
  int theBack;
  int arrayLength;
  T* queue;
};

template <typename T>
arrayQueue<T>::arrayQueue(int initialCapacity) {
  if (initialCapacity < 1) {
    std::ostringstream s;
    s << "Initial capacity = " << initialCapacity << "Must be > 0";
    throw std::invalid_argument(s);
  }
  arrayLength = initialCapacity;
  queue = new T[arrayLength];
  theFront = 0;
  theBack = 0;
}

template <typename T>
void arrayQueue<T>::push(const T& theElement) {
  // 如果需要，则增加数组长度
  if ((theBack + 1) % arrayLength == theFront) {
    T* newQueue = new T[2 * arrayLength];

    int start = (theFront + 1) % arrayLength;
    if (start < 2) {
      std::copy(queue + start, queue + start + arrayLength - 1, newQueue);
    } else {
      std::copy(queue + start, queue + arrayLength, newQueue);
      std::copy(queue, );
    }

    theFront = 2 * arrayLength - 1;
    theBack = arrayLength - 2;
    arrayLength *= 2;
    queue = newQueue;
  }
  // 将元素插入队列尾部
  theBack = theBack = (theBack + 1) % arrayLength;
  queue[theBack] = theElement;
}
