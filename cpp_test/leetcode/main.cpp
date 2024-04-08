#include <iostream>

using namespace std;
class Solution {
 public:
  int coi(char c) {
    int num = c - '0';
    return num;
  }
  string addBinary(string a, string b) {
    std::string s;
    int num_zeros = a.size() - b.size();
    if (num_zeros > 0) {
      b = std::string(num_zeros, '0') + b;
      s = std::string(a.size(), '0');
    } else if (num_zeros == 0) {
      s = std::string(a.size(), '0');
    } else {
      a = std::string(-num_zeros, '0') + a;
      s = std::string(b.size(), '0');
    }
    int plus = 0;
    for (int i = a.size() - 1; i >= 0; i--) {
      int d = coi(a[i]) + coi(b[i]) + plus;
      if (d == 0) {
        s[i] = '0';
        plus = 0;
      } else if (d == 1) {
        s[i] = '1';
        plus = 0;
      } else {
        s[i] = '0';
        plus = 1;
      }
    }
    if (plus == 1) {
      s = "1" + s;
    }
    return s;
  }
};

int main() {
  std::string a = "1";
  std::string b = "111";
  std::cout << "a size " << a.size();
  Solution A;
  std::cout << A.addBinary(a, b);
  return 0;
}