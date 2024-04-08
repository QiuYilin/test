#pragma once
#include "chainNode.hpp"
#include "queue.hpp"

template <typename T>
class linkedQueue : public queue<T> {
 public:
  linkedQueue(int initialCapacity = 10) {}
  ~linkedQueue();
  bool empty() const { return queueSize == 0; }
  int size() const { return queueSize; }
  T& front() {
    if (queueSize == 0) {
      throw std::runtime_error("queue empty");
    }
    return queueBack->element;
  }
  T& back() {
    if (queueSize == 0) {
      throw std::runtime_error("queue empty");
    }
    return queueBack->element;
  }
  void pop();
  void push(const T&);

 private:
  chainNode<T>* queueFront;
  chainNode<T>* queueBack;
  int queueSize;
};

template <typename T>
linkedQueue<T>::~linkedQueue() {
  while (queueFront != NULL) {
    chainNode<T>* nextNode = queueFront->next;
    delete queueFront;
    queueFront = nextNode;
  }
}

template <typename T>
void linkedQueue<T>::pop() {
  if (queueFront == NULL) {
    throw std::runtime_error("queue empty");
  }
  chainNode<T>* nextNode = queueFront->next;
  delete queueFront;
  queueFront = nextNode;
  queueSize--;
}

template <typename T>
void linkedQueue<T>::push(const T& theElement) {
  chainNode<T>* newNode = new chainNode<T>(theElement, NULL);

  if (queueSize == 0) {
    queueFront = newNode;
  } else {
    queueBack->next = newNode;
  }

  queueBack = newNode;
  queueSize++;
}