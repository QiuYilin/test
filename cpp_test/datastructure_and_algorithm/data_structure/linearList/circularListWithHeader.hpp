// circularList list with header node

#ifndef circularListWithHeader_
#define circularListWithHeader_

#include <iostream>
#include <sstream>
#include <string>

#include "chainNode.hpp"
#include "myExceptions.hpp"

using namespace std;

/// @brief 访问节点的灵活性：
// 单向链表：从头结点开始，访问链表中的任何节点都需要遍历整个链表。这会导致时间复杂度为
// O(n)。
// 单向循环链表：从任意节点出发，都能遍历整个列表，因此具有更高的灵活性。这对于某些应用场景非常有用。
//     插入和删除操作的便利性：
//     单向链表：在插入和删除节点时，需要处理头尾节点的特殊情况，因此时间复杂度为
//     O(1)。 单向循环链表：同样具有
//     O(1) 的插入和删除操作，但不需要像单向链表那样处理头尾节点。
// 内存利用率：
//     单向循环链表在某些应用场景中非常有用，例如环形缓冲区。它可以实现高效的资源利用，因为链表的大小可以动态增加或减少，不会浪费内存。
/// @tparam T
template <class T>
class circularListWithHeader {
 public:
  // constructor
  circularListWithHeader();

  // some methods
  int size() const { return listSize; }
  int indexOf(const T& theElement) const;
  void insert(int theIndex, const T& theElement);
  void output(ostream& out) const;

 protected:
  void checkIndex(int theIndex) const;
  // throw illegalIndex if theIndex invalid
  chainNode<T>* headerNode;  // pointer to header node
  int listSize;              // number of elements in list
};

template <class T>
circularListWithHeader<T>::circularListWithHeader() {  // Constructor.
  headerNode = new chainNode<T>();
  headerNode->next = headerNode;
  listSize = 0;
}

template <class T>
void circularListWithHeader<T>::checkIndex(int theIndex)
    const {  // Verify that theIndex is between 0 and listSize - 1.
  if (theIndex < 0 || theIndex >= listSize) {
    ostringstream s;
    s << "index = " << theIndex << " size = " << listSize;
    throw illegalIndex(s.str());
  }
}

template <class T>
int circularListWithHeader<T>::indexOf(const T& theElement)
    const {  // Return index of first occurrence of theElement.
             // Return -1 if theElement not in list.

  // Put theElement in header node
  headerNode->element = theElement;  // 把目标元素放到头节点中

  // search the chain for theElement
  chainNode<T>* currentNode = headerNode->next;
  int index = 0;  // index of currentNode
  while (currentNode->element != theElement) {
    // move to next node
    currentNode = currentNode->next;
    index++;
  }

  // make sure we found matching element
  if (currentNode == headerNode)
    return -1;
  else
    return index;
}

template <class T>
void circularListWithHeader<T>::insert(
    int theIndex,
    const T& theElement) {  // Insert theElement so that its index is theIndex.
  if (theIndex < 0 || theIndex > listSize) {  // invalid index
    ostringstream s;
    s << "index = " << theIndex << " size = " << listSize;
    throw illegalIndex(s.str());
  }

  // find predecessor of new element
  chainNode<T>* p = headerNode;
  for (int i = 0; i < theIndex; i++) p = p->next;

  // insert after p
  p->next = new chainNode<T>(theElement, p->next);

  listSize++;
}

template <class T>
void circularListWithHeader<T>::output(
    ostream& out) const {  // Put the list into the stream out.
  for (chainNode<T>* currentNode = headerNode->next; currentNode != headerNode;
       currentNode = currentNode->next)
    out << currentNode->element << "  ";
}

// overload <<
template <class T>
ostream& operator<<(ostream& out, const circularListWithHeader<T>& x) {
  x.output(out);
  return out;
}

#endif
