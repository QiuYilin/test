#pragma once

template <typename K, typename E>
struct pairNode {
  typedef std::pair<const K, E> pairType;
  pairType element;
  pairNode<K, E>* next;  // 指向同类型的指针

  pairNode(const pairType& thePair) : element(thePair) {}
  pairNode(const pairType& thePair, pairNode<K, E>* theNext)
      : element(thePair) {
    next = theNext;
  }
};