#pragma once

/// @brief 迭代器
/// @tparam T
template <typename T>
class vertexIterator {
 public:
  ~vertexIterator() {}
  virtual int next() = 0;
  virtual int next(T &) = 0;
};

template <typename T>
class edge {
 public:
  virtual ~edge() {}
  vertexIterator<T> *vertex1();
  vertexIterator<T> *vertex2();
  T weight();
};

template <typename T>
class graph {
 public:
  virtual ~graph() {}

  // ADT 方法
  virtual int numberOfVertices() const = 0;
  virtual int numberOfEdges() const = 0;
  virtual bool existEdge(int, int) const = 0;
  virtual void insertEdge(edge<T> *) = 0;
  virtual void eraseEdge(int, int) = 0;
  virtual int degree(int) const = 0;
  virtual int inDegree(int) const = 0;
  virtual int outDegree(int) const = 0;

  // 其他方法
  virtual bool directed() const = 0;
  virtual bool weighted() const = 0;
  virtual vertexIterator<T> *iterator(int) = 0;
};
