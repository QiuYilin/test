#ifndef LIST_H
#define LIST_H

#include <algorithm>

/// @brief 双向循环链表 包含连接到表两端的链，表大小，以及一些方法
/// @tparam Object
template <typename Object>
class List {
 private:
  // The basic doubly linked list node.
  // Nested inside of List, can be public
  // because the Node is itself private
  // 表示一种主要包含直接被存取而不是通过方法访问的数据的类
  // 包含数据和指向前后两个节点的两个指针，以及适当的构造函数
  struct Node {
    Object data;
    Node *prev;
    Node *next;

    Node(const Object &d = Object{}, Node *p = nullptr, Node *n = nullptr)
        : data{d}, prev{p}, next{n} {}

    Node(Object &&d, Node *p = nullptr, Node *n = nullptr)
        : data{std::move(d)}, prev{p}, next{n} {}
  };

 public:
  /// @brief 抽象位置概念，公有的内嵌类，存储一个指向当前节点的指针
  //  并且提供基本迭代器操作的实现
  class const_iterator {
   public:
    using value_type = Object;
    using difference_type = std::ptrdiff_t;
    using pointer = Object *;
    using reference = const Object &;
    using iterator_category = std::bidirectional_iterator_tag;
    // Public constructor for const_iterator.
    const_iterator() : current{nullptr} {}

    // Return the object stored at the current position.
    // For const_iterator, this is an accessor with a
    // const reference return type.
    const Object &operator*() const { return retrieve(); }

    const_iterator &operator++() {
      current = current->next;
      return *this;
    }

    const_iterator operator++(int) {
      const_iterator old = *this;
      ++(*this);
      return old;
    }

    const_iterator &operator--() {
      current = current->prev;
      return *this;
    }

    const_iterator operator--(int) {
      const_iterator old = *this;
      --(*this);
      return old;
    }

    bool operator==(const const_iterator &rhs) const {
      return current == rhs.current;
    }

    bool operator!=(const const_iterator &rhs) const { return !(*this == rhs); }

   protected:
    const List<Object> *theList;
    Node *current;

    // Protected helper in const_iterator that returns the object
    // stored at the current position. Can be called by all
    // three versions of operator* without any type conversions.
    Object &retrieve() const { return current->data; }

    // Protected constructor for const_iterator.
    // Expects a pointer that represents the current position.
    const_iterator(const List<Object> &lst, Node *p)
        : theList{&lst}, current{p} {}

    void assertValid() const {
      if (theList == nullptr || current == nullptr ||
          current == theList->head) {
        throw std::runtime_error("iterator out of bounds");
      }
    }
    friend class List<Object>;
  };

  /// @brief iterator是一个const_iterator；返回所指该项的引用
  class iterator : public const_iterator {
   public:
    using value_type = Object;
    using difference_type = std::ptrdiff_t;
    using pointer = Object *;
    using reference = Object &;
    using iterator_category = std::bidirectional_iterator_tag;
    // Public constructor for iterator.
    // Calls the base-class constructor.
    // Must be provided because the private constructor
    // is written; otherwise zero-parameter constructor
    // would be disabled.
    iterator() {}

    Object &operator*() { return const_iterator::retrieve(); }

    // Return the object stored at the current position.
    // For iterator, there is an accessor with a
    // const reference return type and a mutator with
    // a reference return type. The accessor is shown first.
    const Object &operator*() const { return const_iterator::operator*(); }

    iterator &operator++() {
      this->current = this->current->next;
      return *this;
    }

    iterator operator++(int) {
      iterator old = *this;
      ++(*this);
      return old;
    }

    iterator &operator--() {
      this->current = this->current->prev;
      return *this;
    }

    iterator operator--(int) {
      iterator old = *this;
      --(*this);
      return old;
    }

   protected:
    // Protected constructor for iterator.
    // Expects the current position.
    iterator(const List<Object> &lst, Node *p) : const_iterator{lst, p} {}

    friend class List<Object>;
  };

 public:
  List() { init(); }

  ~List() {
    clear();
    delete head;
    delete tail;
  }

  List(const List &rhs) {
    init();
    for (auto &x : rhs) push_back(x);
  }

  List &operator=(const List &rhs) {
    List copy = rhs;
    std::swap(*this, copy);
    return *this;
  }

  List(List &&rhs) : theSize{rhs.theSize}, head{rhs.head}, tail{rhs.tail} {
    rhs.theSize = 0;
    rhs.head = nullptr;
    rhs.tail = nullptr;
  }

  List &operator=(List &&rhs) {
    std::swap(theSize, rhs.theSize);
    std::swap(head, rhs.head);
    std::swap(tail, rhs.tail);

    return *this;
  }

  // Return iterator representing beginning of list.
  // Mutator version is first, then accessor version.
  iterator begin() { return iterator(*this, head->next); }

  const_iterator begin() const { return const_iterator(head->next); }

  // Return iterator representing endmarker of list.
  // Mutator version is first, then accessor version.
  iterator end() { return iterator(*this, tail); }

  const_iterator end() const { return const_iterator(tail); }

  // Return number of elements currently in the list.
  int size() const { return theSize; }

  // Return true if the list is empty, false otherwise.
  bool empty() const { return size() == 0; }

  /// @brief 反复删除成员项直到List为空
  void clear() {
    while (!empty()) pop_front();
  }

  // front, back, push_front, push_back, pop_front, and pop_back
  // are the basic double-ended queue operations.
  Object &front() { return *begin(); }

  const Object &front() const { return *begin(); }

  Object &back() { return *--end(); }

  const Object &back() const { return *--end(); }

  void push_front(const Object &x) { insert(begin(), x); }

  void push_back(const Object &x) { insert(end(), x); }

  void push_front(Object &&x) { insert(begin(), std::move(x)); }

  void push_back(Object &&x) { insert(end(), std::move(x)); }

  /// @brief 回收节点
  void pop_front() { erase(begin()); }

  void pop_back() { erase(--end()); }

  // Insert x before itr.
  iterator insert(iterator itr, const Object &x) {
    itr.assertValid();
    if (itr.theList != this) {
      throw std::runtime_error("iterator mismatch");
    }
    Node *p = itr.current;
    ++theSize;
    auto p_new = new Node{x, p->prev, p};
    p->prev->next = p_new;
    p->prev = p_new;
    return iterator(p_new);
  }

  // Insert x before itr.
  iterator insert(iterator itr, Object &&x) {
    itr.assertValid();
    if (itr.theList != this) {
      throw std::runtime_error("iterator mismatch");
    }
    Node *p = itr.current;
    ++theSize;

    return iterator(
        *this, p->prev = p->prev->next = new Node{std::move(x), p->prev, p});
  }

  // Erase item at itr.
  iterator erase(iterator itr) {
    itr.assertValid();
    if (itr.theList != this) {
      throw std::runtime_error("iterator mismatch");
    }
    Node *p = itr.current;
    iterator retVal(*this, p->next);
    p->prev->next = p->next;
    p->next->prev = p->prev;
    delete p;
    --theSize;

    return retVal;
  }

  iterator erase(iterator from, iterator to) {
    from.assertValid();
    to.assertValid();
    if (from.theList != this || to.theList != this) {
      throw std::runtime_error("iterator mismatch");
    }

    for (iterator itr = from; itr != to;) itr = erase(itr);

    return to;
  }

 private:
  int theSize;
  /// @brief 头节点的好处,删除头节点不再是特殊情况
  Node *head;
  Node *tail;

  void init() {
    theSize = 0;
    head = new Node;
    tail = new Node;
    head->next = tail;
    tail->prev = head;
  }
};

#endif
