#pragma once
#include <iostream>

template <typename T>
class Matrix {
  template <
      typename F>  // 先声明再定义必须使用不同模板参数，忽略或者使用同样的模板参数是不可行的
  friend std::ostream& operator<<(std::ostream&, const Matrix<F>&);

 public:
  Matrix(int the_rows = 0, int the_columns = 0);
  Matrix(const Matrix<T>&);
  ~Matrix() { delete[] _element; }
  int rows() const { return _the_rows; }
  int columns() const { return _the_columns; }
  T& operator()(int i, int j) const;
  Matrix<T>& operator=(const Matrix<T>&);
  Matrix<T> operator+() const;  // unary +
  Matrix<T> operator+(const Matrix<T>&) const;
  Matrix<T> operator-() const;  // unary minus
  Matrix<T> operator-(const Matrix<T>&) const;
  Matrix<T> operator*(const Matrix<T>&) const;
  Matrix<T>& operator+=(const T&);

 private:
  int _the_rows,     // number of rows in Matrix
      _the_columns;  // number of columns in Matrix
  T* _element;       // element array
};

template <typename T>
Matrix<T>::Matrix(int the_rows, int the_columns) {  // Matrix constructor.
  // validate theRows and theColumns
  if (the_rows < 0 || the_columns < 0)
    throw std::invalid_argument("Rows and columns must be >= 0");
  if ((the_rows == 0 || the_columns == 0) &&
      (the_rows != 0 || the_columns != 0))
    throw std::invalid_argument(
        "Either both or neither rows and columns should be zero");

  // create the Matrix
  _the_rows = the_rows;
  _the_columns = the_columns;
  _element = new T[the_rows * the_columns];
}

template <typename T>
Matrix<T>::Matrix(const Matrix<T>& m) {  // Copy constructor for matrices.
  // create Matrix
  _the_rows = m._the_rows;
  _the_columns = m._the_columns;
  _element = new T[_the_rows * _the_columns];

  // copy each element of m
  std::copy(m._element, m._element + _the_rows * _the_columns, _element);
}

template <typename T>
Matrix<T>& Matrix<T>::operator=(
    const Matrix<T>& m) {  // Assignment. (*this) = m.
  if (this != &m) {        // not copying to self
    delete[] _element;
    _the_rows = m._the_rows;
    _the_columns = m._the_columns;
    _element = new T[_the_rows * _the_columns];
    // copy each element
    std::copy(m._element, m._element + _the_rows * _the_columns, _element);
  }
  return *this;
}

template <typename T>
T& Matrix<T>::operator()(int i,
                         int j) const {  // Return a reference to element (i,j).
  if (i < 0 || i >= _the_rows || j < 0 || j >= _the_columns)
    throw std::out_of_range("Matrix index out of bounds");
  return _element[i * _the_columns + j];
}

template <typename T>
Matrix<T> Matrix<T>::operator+(
    const Matrix<T>& m) const {  // Return w = (*this) + m.
  if (_the_rows != m._the_rows || _the_columns != m._the_columns)
    throw std::invalid_argument("The size of the two matrics doesn't match");

  // create result Matrix w
  Matrix<T> w(_the_rows, _the_columns);
  for (int i = 0; i < _the_rows * _the_columns; i++)
    w._element[i] = _element[i] + m._element[i];

  return w;
}

template <class T>
Matrix<T> Matrix<T>::operator-(
    const Matrix<T>& m) const {  // Return (*this) - m.
  if (_the_rows != m._the_rows || _the_columns != m._the_columns)
    throw matrixSizeMismatch();

  // create result Matrix w
  Matrix<T> w(_the_rows, _the_columns);
  for (int i = 0; i < _the_rows * _the_columns; i++)
    w._element[i] = _element[i] - m._element[i];

  return w;
}

template <class T>
Matrix<T> Matrix<T>::operator-() const {  // Return w = -(*this).

  // create result Matrix w
  Matrix<T> w(_the_rows, _the_columns);
  for (int i = 0; i < _the_rows * _the_columns; i++)
    w._element[i] = -_element[i];
  return w;
}

template <typename T>
Matrix<T> Matrix<T>::operator*(
    const Matrix<T>& m) const {  // Matrix multiply.  Return w = (*this) * m.
  if (_the_columns != m._the_rows)
    throw std::invalid_argument("The size of the two matrics doesn't match");

  Matrix<T> w(_the_rows, m._the_columns);  // result Matrix

  // define cursors for *this, m, and w
  // and initialize to location of (1,1) element
  int ct = 0, cm = 0, cw = 0;

  // compute w(i,j) for all i and j+
  for (int i = 1; i <= _the_rows; i++) {         // compute row i of result
    for (int j = 1; j <= m._the_columns; j++) {  // compute first term of w(i,j)
      T sum = _element[ct] * m._element[cm];

      // add in remaining terms
      for (int k = 2; k <= _the_columns; k++) {
        ct++;                  // next term in row i of *this
        cm += m._the_columns;  // next in column j of m
        sum += _element[ct] * m._element[cm];
      }
      w._element[cw++] = sum;  // save w(i,j)

      // reset to start of row and next column
      ct -= _the_columns - 1;
      cm = j;
    }

    // reset to start of next row and first column
    ct += _the_columns;
    cm = 0;
  }

  return w;
}

template <typename T>
Matrix<T>& Matrix<T>::operator+=(
    const T& x) {  // Increment all elements of *this by x.
  for (int i = 0; i < _the_rows * _the_columns; i++) {
    _element[i] += x;
  }
  return *this;
}

template <typename F>
std::ostream& operator<<(std::ostream& out,
                         const Matrix<F>& m) {  // Put Matrix m into the stream
  // out. One row per line.
  int k = 0;                               // index into element array
  for (int i = 0; i < m._the_rows; i++) {  // do row i
    for (int j = 0; j < m._the_columns; j++) {
      out << m._element[k++] << "  ";
    }
    // row i finished
    out << std::endl;
  }
  return out;
}
