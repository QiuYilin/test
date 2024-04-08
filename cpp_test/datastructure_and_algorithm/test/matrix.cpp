// test matrix class

#include "../data_structure/matrix/matrix.hpp"

#include <iostream>

using namespace std;
int main(void) {
  try {
    Matrix<int> x{3, 2}, y, z;
    int i, j;
    for (i = 0; i <= 2; i++) {
      for (j = 0; j <= 1; j++) {
        x(i, j) = 2 * i + j;
      }
    }

    cout << "Initialized x(i,j) = 2*i + j" << endl;
    cout << "x(2,1) = " << x(2, 1) << endl;
    cout << "x is" << endl;
    cout << x << endl;

    y = x;
    cout << "Assigned y = x" << endl;
    cout << "y is" << endl;
    cout << y << endl;

    x += 2;
    cout << "x incremented by 2 is" << endl;
    cout << x << endl;

    z = y + x;
    cout << "y + x is" << endl;
    cout << z << endl;

    cout << "-(y + x) is" << endl;
    cout << -z << endl;

    Matrix<int> w{2, 3};
    for (i = 0; i <= 1; i++)
      for (j = 0; j <= 2; j++) {
        w(i, j) = i + j;
      }
    cout << "Initialized w(i,j) = i + j" << endl;
    cout << "w is" << endl;
    cout << w << endl;

    z = y * w;
    cout << "y * w is" << endl;
    cout << z << endl;
  } catch (...) {
    cerr << "An exception has occurred" << endl;
  }

  return 0;
}
